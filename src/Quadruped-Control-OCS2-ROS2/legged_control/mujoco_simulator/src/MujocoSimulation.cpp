#include "mujoco_simulator/MujocoSimulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

MujocoSimulation* MujocoSimulation::instance_ = nullptr;
bool MujocoSimulation::button_left = false;
bool MujocoSimulation::button_middle = false;
bool MujocoSimulation::button_right = false;
double MujocoSimulation::lastx = 0.0;
double MujocoSimulation::lasty = 0.0;

namespace {
constexpr std::array<const char*, 5> kDisturbanceBodyNames = {
    "trunk", "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"
};

constexpr std::array<const char*, 5> kDisturbanceBodyLabels = {
    "trunk", "LF foot", "RF foot", "LH foot", "RH foot"
};
}  // namespace

MujocoSimulation::MujocoSimulation(const rclcpp::Node::SharedPtr& node,
    const std::string& xmlFile,
    const std::string& simulatorFile,
    bool exposeRosInterface)
    : node_(node), model_(nullptr), data_(nullptr), window_(nullptr),
      exposeRosInterface_(exposeRosInterface),
      disturbance_rng_(static_cast<std::mt19937::result_type>(std::time(nullptr)))
{   
    instance_ = this;
    // Initialize MuJoCo visualization
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // Create window
    window_ = glfwCreateWindow(1280, 720, "MuJoCo Simulation", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        mju_error("Could not create GLFW window");
    }
    glfwMakeContextCurrent(window_);
    glfwShowWindow(window_);
    glfwSwapInterval(0);  // Disable vsync so rendering is not capped by display refresh

     // Initialize MuJoCo rendering
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);

    // load model
    loadModel(xmlFile, simulatorFile);
    render_rtf_window_start_sim_time_ = data_ ? data_->time : 0.0;

    // set redering;
    renderSetting(simulatorFile);

    if (exposeRosInterface_) {
        state_pub_ = node_->create_publisher<legged_msgs::msg::SimulatorStateData>("simulator_state_data", 1);
        sensor_pub_ = node_->create_publisher<legged_msgs::msg::SimulatorSensorData>("simulator_sensor_data", 1);
        joint_control_sub_ = node_->create_subscription<legged_msgs::msg::JointControlData>(
            "joint_control_data", 1, std::bind(&MujocoSimulation::control_callback, this, std::placeholders::_1));
        emergency_override_state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "legged_robot_emergency_override_state", 1,
            std::bind(&MujocoSimulation::emergencyOverrideStateCallback, this, std::placeholders::_1));
        start_control_server_ = node_->create_service<legged_msgs::srv::StartControl>("start_control",
                std::bind(&MujocoSimulation::start_control_service, this, std::placeholders::_1, std::placeholders::_2));
    }

}

MujocoSimulation::~MujocoSimulation() {
    // Free MuJoCo resources
    if (model_) mj_deleteModel(model_);
    if (data_) mj_deleteData(data_);

    // Free MuJoCo rendering context
    mjr_freeContext(&context_);

    // Terminate GLFW
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

void MujocoSimulation::loadModel(const std::string& modelPath, const std::string& configFile) {
    // controller
    ocs2::loadData::loadCppDataType(configFile, "controller.timestep", timestep_);
    ocs2::loadData::loadCppDataType(configFile, "controller.wbc_control_frequency", control_frequency_);

    // pid
    ocs2::loadData::loadCppDataType(configFile, "pid.kp", Kp_);
    ocs2::loadData::loadCppDataType(configFile, "pid.kd", Kd_);
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(configFile, pt);
    estopKp_ = pt.get<double>("pid.estop_kp", estopKp_);
    estopKd_ = pt.get<double>("pid.estop_kd", estopKd_);
    estopKp_ = std::max(Kp_, estopKp_);
    estopKd_ = std::max(Kd_, estopKd_);

    // disturbance with optional config override
    disturbance_force_min_ = pt.get<double>("disturbance.force_min", disturbance_force_min_);
    disturbance_force_max_ = pt.get<double>("disturbance.force_max", disturbance_force_max_);
    disturbance_vertical_scale_ = pt.get<double>("disturbance.vertical_scale", disturbance_vertical_scale_);
    disturbance_interval_min_ = pt.get<double>("disturbance.interval_min", disturbance_interval_min_);
    disturbance_interval_max_ = pt.get<double>("disturbance.interval_max", disturbance_interval_max_);
    disturbance_impulse_duration_ = pt.get<double>("disturbance.impulse_duration", disturbance_impulse_duration_);
    disturbance_arrow_scale_ = pt.get<double>("disturbance.arrow_scale", disturbance_arrow_scale_);
    disturbance_force_min_ = std::max(0.0, disturbance_force_min_);
    disturbance_force_max_ = std::max(disturbance_force_min_, disturbance_force_max_);
    disturbance_vertical_scale_ = std::max(0.0, disturbance_vertical_scale_);
    disturbance_interval_min_ = std::max(timestep_, disturbance_interval_min_);
    disturbance_interval_max_ = std::max(disturbance_interval_min_, disturbance_interval_max_);
    disturbance_impulse_duration_ = std::max(timestep_, disturbance_impulse_duration_);
    disturbance_arrow_scale_ = std::max(0.0, disturbance_arrow_scale_);
    disturbance_force_scales_[0] = std::max(0.0, pt.get<double>("disturbance.trunk_force_scale", disturbance_force_scales_[0]));
    disturbance_force_scales_[1] = std::max(0.0, pt.get<double>("disturbance.lf_foot_force_scale", disturbance_force_scales_[1]));
    disturbance_force_scales_[2] = std::max(0.0, pt.get<double>("disturbance.rf_foot_force_scale", disturbance_force_scales_[2]));
    disturbance_force_scales_[3] = std::max(0.0, pt.get<double>("disturbance.lh_foot_force_scale", disturbance_force_scales_[3]));
    disturbance_force_scales_[4] = std::max(0.0, pt.get<double>("disturbance.rh_foot_force_scale", disturbance_force_scales_[4]));

    // Load MuJoCo model and data
    model_ = mj_loadXML(modelPath.c_str(), nullptr, nullptr, 0);
    if (!model_) {
        mju_error("Failed to load model");
    }
    model_->opt.timestep = timestep_; //simulation timestep

    data_ = mj_makeData(model_);
    if (model_->nq >= 7) {
        initial_base_quat_[0] = data_->qpos[3];
        initial_base_quat_[1] = data_->qpos[4];
        initial_base_quat_[2] = data_->qpos[5];
        initial_base_quat_[3] = data_->qpos[6];
    }
    initial_qpos_.assign(data_->qpos, data_->qpos + model_->nq);
    initial_qvel_.assign(data_->qvel, data_->qvel + model_->nv);

    mjv_makeScene(model_, &scene_, 2000); 
    // Create MuJoCo context for rendering
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_WINDOW, &context_);
    if (context_.currentBuffer != mjFB_WINDOW) {
        RCLCPP_WARN(node_->get_logger(),
                    "MuJoCo renderer could not select the window framebuffer during initialization.");
    }

    for (int i = 0; i < 12; ++i) {
        Joint_position_[i] = 0.0;
        Joint_velocity_[i] = 0.0;
        Joint_torque_[i] = 0.0;
    }

    if (!setDisturbanceBody(kDisturbanceBodyNames[0], kDisturbanceBodyLabels[0])) {
        RCLCPP_WARN(node_->get_logger(), "Base disturbance body 'trunk' not found. Disturbance toggle will be disabled.");
    }
    
    Start_simulate_=true;
    Start_control_=false;

    std::vector<std::string> geom_names = {"floor", "LF_FOOT_geom", "RF_FOOT_geom", "LH_FOOT_geom", "RH_FOOT_geom"};
    for (const auto& name : geom_names) {
        int id = mj_name2id(model_, mjOBJ_GEOM, name.c_str());
        if (id != -1) {
            geom_ids_.push_back(id);
        }
    }
    for (int id : geom_ids_) {
        std::cout << "Geom ID: " << id << std::endl;
    }

}

void MujocoSimulation::renderSetting(const std::string& configFile)
{
    ocs2::loadData::loadCppDataType(configFile, "render.render_appearance", render_appearance_);
    ocs2::loadData::loadCppDataType(configFile, "render.render_inertia", render_inertia_);
    ocs2::loadData::loadCppDataType(configFile, "render.visualize_contacts", visualize_contacts_);
    ocs2::loadData::loadCppDataType(configFile, "render.tracking_base", tracking_base_);
    ocs2::loadData::loadCppDataType(configFile, "render.render_frequency", render_frequency_);

    if (model_ != nullptr) {
        mjv_defaultFreeCamera(model_, &cam_);
    }

    if (tracking_base_) {
        const char* object_name = "trunk";
        int object_body_id = mj_name2id(model_, mjOBJ_BODY, object_name);
        if (object_body_id >= 0) {
            cam_.type = mjCAMERA_TRACKING;
            cam_.trackbodyid = object_body_id;
            cam_.distance = 1.0;
            cam_.azimuth = 135.0;
            cam_.elevation = -20.0;
        }
    }
    if (visualize_contacts_) {
        opt_.flags[mjVIS_CONTACTPOINT] = 1; 
    }
    for (int i = 0; i < 3; ++i) {
        opt_.geomgroup[i] = 0;
    }
    opt_.geomgroup[0] = 1;
    if (render_appearance_) {
        opt_.geomgroup[1] = 1;
    }
    if (render_inertia_) {
        opt_.geomgroup[2] = 1;
    }

    glfwSetKeyCallback(window_, keyboard);
    glfwSetCursorPosCallback(window_, mouse_move);
    glfwSetMouseButtonCallback(window_, mouse_button);
    glfwSetScrollCallback(window_, scroll);
}

void MujocoSimulation::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    if (act != GLFW_PRESS || instance_ == nullptr) {
        return;
    }

    if (key == GLFW_KEY_BACKSPACE) {
        mj_resetData(instance_->model_, instance_->data_);
        instance_->clearDisturbanceForce();
        instance_->disturbance_enabled_ = false;
        instance_->clearActuatorCommandState();
        mj_forward(instance_->model_, instance_->data_);
        return;
    }

    if (key == GLFW_KEY_R) {
        instance_->resetRobotPose();
        RCLCPP_INFO(instance_->node_->get_logger(),
                    "Robot reset to start pose at x=0.00 y=0.00 z=0.30.");
        return;
    }

    if (key == GLFW_KEY_D) {
        if (instance_->disturbance_body_id_ < 0) {
            RCLCPP_WARN(instance_->node_->get_logger(), "Cannot toggle disturbances because the selected body was not found.");
            return;
        }

        instance_->disturbance_enabled_ = !instance_->disturbance_enabled_;
        if (instance_->disturbance_enabled_) {
            instance_->clearDisturbanceForce();
            instance_->next_disturbance_update_time_ =
                instance_->data_ ? instance_->data_->time : 0.0;
            RCLCPP_INFO(instance_->node_->get_logger(),
                        "Random impulses enabled on %s. Press 0 for trunk, 1-4 for feet, or 'd' again to turn them off.",
                        instance_->getDisturbanceBodyLabel());
        } else {
            instance_->clearDisturbanceForce();
            RCLCPP_INFO(instance_->node_->get_logger(), "Random impulses disabled.");
        }
        return;
    }

    int disturbanceSelectionIndex = -1;
    switch (key) {
        case GLFW_KEY_0:
            disturbanceSelectionIndex = 0;
            break;
        case GLFW_KEY_1:
            disturbanceSelectionIndex = 1;
            break;
        case GLFW_KEY_2:
            disturbanceSelectionIndex = 2;
            break;
        case GLFW_KEY_3:
            disturbanceSelectionIndex = 3;
            break;
        case GLFW_KEY_4:
            disturbanceSelectionIndex = 4;
            break;
        default:
            break;
    }

    if (disturbanceSelectionIndex >= 0) {
        const char* targetBodyName = kDisturbanceBodyNames[disturbanceSelectionIndex];
        const char* targetBodyLabel = kDisturbanceBodyLabels[disturbanceSelectionIndex];
        if (instance_->setDisturbanceBody(targetBodyName, targetBodyLabel)) {
            RCLCPP_INFO(instance_->node_->get_logger(),
                        "Disturbance target set to %s. Press 'd' to %s disturbances.",
                        instance_->getDisturbanceBodyLabel(),
                        instance_->disturbance_enabled_ ? "keep applying" : "enable");
        } else {
            RCLCPP_WARN(instance_->node_->get_logger(),
                        "Could not select disturbance target '%s' because that body was not found in the model.",
                        targetBodyName);
        }
    }
}

void MujocoSimulation::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);
}

void MujocoSimulation::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    if (!button_left && !button_middle && !button_right)
        return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(instance_->model_, action, dx / height, dy / height, &instance_->scene_, &instance_->cam_);
}

void MujocoSimulation::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    mjv_moveCamera(instance_->model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &instance_->scene_, &instance_->cam_);
}


void MujocoSimulation::run()
{
    rclcpp::Rate loop_rate(1000);
    simulateStep();
    render();

    mjtNum last_render_time = data_->time;
    int Render_count_=0;

    while (rclcpp::ok()) {
        // Render
        if (Render_count_>=50)
        {
            render();
            Render_count_=0;
        }
        Render_count_+=1;

        // Perform simulation step
        if (Start_control_) {
            // Reset *mujoco_sim.getSimuState()
            //mujoco_sim.render();

            Start_control_ = false;
            mjtNum simstart = data_->time;

            // Perform simulation for 1.0 / Control_frequency_ seconds
            while (data_->time - simstart < 1.0 / control_frequency_) {
                simulateStep();
                
            }
            // Render while robot is running
            if (data_->time - last_render_time >= 1.0 / render_frequency_) {
                render();
                last_render_time = data_->time;
            }
            
        }
        else if (Start_simulate_)
        {
            render();
            mjtNum simstart = data_->time;
            // Perform simulation for 1.0 / 100.0 seconds
            while (data_->time - simstart < 1.0 / control_frequency_) {
                simulateStep();
            }
            // Render while robot is running
            if (data_->time - last_render_time >= 1.0 / 100.0) {
                render();
                last_render_time = data_->time;
            }
        }
        rclcpp::spin_some(node_);
        loop_rate.sleep(); // Maintain 100Hz loop
    }

}

void MujocoSimulation::simulateStep() {
    updateDisturbanceForce();

    if (!Start_simulate_)
    {
        const bool strongPdMode = actuator_mode_ == 1;
        const bool zeroTorqueMode = actuator_mode_ == 2;
        const double effectiveKp = strongPdMode ? estopKp_ : Kp_;
        const double effectiveKd = strongPdMode ? estopKd_ : Kd_;
        double joint_position_value[12];
        double joint_velocity_value[12];
        double control_torque[12];
        //control sequence: FR,FL,RR,RL(hip, thigh, calf)
        for (int i = 0; i < 12; ++i) {
            joint_position_value[i] = data_->sensordata[i+10];
            joint_velocity_value[i] = data_->sensordata[i+22];
            if (zeroTorqueMode) {
                control_torque[i] = 0.0;
            } else {
                control_torque[i] = static_cast<double>(Joint_torque_[i]) +
                    effectiveKp * (static_cast<double>(Joint_position_[i]) - joint_position_value[i]) +
                    effectiveKd * (static_cast<double>(Joint_velocity_[i]) - joint_velocity_value[i]);
            }
            data_->ctrl[i] = control_torque[i];
            //std::cout << "data_->ctrl[" << i << "] = " << data_->ctrl[i] << std::endl;
        }
    }
    // Perform a simulation step
    mj_step(model_, data_);
}

void MujocoSimulation::clearDisturbanceForce() {
    current_disturbance_wrench_.fill(0.0);
    disturbance_active_until_time_ = data_ ? data_->time : 0.0;
    if (data_ == nullptr || disturbance_body_id_ < 0) {
        return;
    }

    double* appliedWrench = data_->xfrc_applied + 6 * disturbance_body_id_;
    for (int i = 0; i < 6; ++i) {
        appliedWrench[i] = 0.0;
    }
}

bool MujocoSimulation::setDisturbanceBody(const char* bodyName, const char* bodyLabel) {
    if (model_ == nullptr || bodyName == nullptr || bodyLabel == nullptr) {
        return false;
    }

    int selectionIndex = -1;
    for (std::size_t i = 0; i < kDisturbanceBodyNames.size(); ++i) {
        if (std::strcmp(bodyName, kDisturbanceBodyNames[i]) == 0) {
            selectionIndex = static_cast<int>(i);
            break;
        }
    }
    if (selectionIndex < 0) {
        return false;
    }

    const int newBodyId = mj_name2id(model_, mjOBJ_BODY, bodyName);
    if (newBodyId < 0) {
        return false;
    }

    if (data_ != nullptr && disturbance_body_id_ >= 0) {
        double* previousAppliedWrench = data_->xfrc_applied + 6 * disturbance_body_id_;
        for (int i = 0; i < 6; ++i) {
            previousAppliedWrench[i] = 0.0;
        }
    }

    disturbance_body_id_ = newBodyId;
    disturbance_body_selection_index_ = selectionIndex;
    disturbance_body_label_ = bodyLabel;
    current_disturbance_wrench_.fill(0.0);
    last_disturbance_wrench_.fill(0.0);
    disturbance_active_until_time_ = data_ ? data_->time : 0.0;
    next_disturbance_update_time_ = data_ ? data_->time : 0.0;
    return true;
}

const char* MujocoSimulation::getDisturbanceBodyLabel() const {
    return disturbance_body_label_.c_str();
}

double MujocoSimulation::getCurrentDisturbanceForceScale() const {
    const std::size_t index = static_cast<std::size_t>(std::clamp(
        disturbance_body_selection_index_, 0, static_cast<int>(disturbance_force_scales_.size() - 1)));
    return disturbance_force_scales_[index];
}

void MujocoSimulation::scheduleNextDisturbance() {
    if (data_ == nullptr) {
        return;
    }

    std::uniform_real_distribution<double> intervalDistribution(
        disturbance_interval_min_, disturbance_interval_max_);
    next_disturbance_update_time_ = data_->time + intervalDistribution(disturbance_rng_);
}

void MujocoSimulation::sampleDisturbanceForce() {
    if (data_ == nullptr || disturbance_body_id_ < 0) {
        return;
    }

    constexpr double kPi = 3.14159265358979323846;
    std::uniform_real_distribution<double> angleDistribution(-kPi, kPi);
    std::uniform_real_distribution<double> magnitudeDistribution(
        disturbance_force_min_, disturbance_force_max_);
    std::uniform_real_distribution<double> verticalDistribution(
        -disturbance_vertical_scale_, disturbance_vertical_scale_);

    const double forceScale = getCurrentDisturbanceForceScale();
    const double planarMagnitude = forceScale * magnitudeDistribution(disturbance_rng_);
    const double planarAngle = angleDistribution(disturbance_rng_);

    current_disturbance_wrench_[0] = planarMagnitude * std::cos(planarAngle);
    current_disturbance_wrench_[1] = planarMagnitude * std::sin(planarAngle);
    current_disturbance_wrench_[2] =
        forceScale * magnitudeDistribution(disturbance_rng_) * verticalDistribution(disturbance_rng_);
    current_disturbance_wrench_[3] = 0.0;
    current_disturbance_wrench_[4] = 0.0;
    current_disturbance_wrench_[5] = 0.0;
    last_disturbance_wrench_ = current_disturbance_wrench_;

    double* appliedWrench = data_->xfrc_applied + 6 * disturbance_body_id_;
    for (int i = 0; i < 6; ++i) {
        appliedWrench[i] = current_disturbance_wrench_[i];
    }
    disturbance_active_until_time_ = data_->time + disturbance_impulse_duration_;

    RCLCPP_INFO(node_->get_logger(),
                "Applied impulse on %s (scale %.2f): fx=%.2f fy=%.2f fz=%.2f duration=%.3f",
                getDisturbanceBodyLabel(),
                forceScale,
                current_disturbance_wrench_[0], current_disturbance_wrench_[1],
                current_disturbance_wrench_[2], disturbance_impulse_duration_);
}

void MujocoSimulation::updateDisturbanceForce() {
    if (!disturbance_enabled_) {
        if (disturbance_body_id_ >= 0) {
            clearDisturbanceForce();
        }
        return;
    }

    if (data_ == nullptr || disturbance_body_id_ < 0) {
        return;
    }

    const bool impulseActive = data_->time < disturbance_active_until_time_;
    if (impulseActive) {
        double* appliedWrench = data_->xfrc_applied + 6 * disturbance_body_id_;
        for (int i = 0; i < 6; ++i) {
            appliedWrench[i] = current_disturbance_wrench_[i];
        }
        return;
    }

    const double currentForceNorm = std::abs(current_disturbance_wrench_[0]) +
                                    std::abs(current_disturbance_wrench_[1]) +
                                    std::abs(current_disturbance_wrench_[2]);
    if (currentForceNorm > 1e-9) {
        clearDisturbanceForce();
        scheduleNextDisturbance();
        return;
    }

    if (data_->time >= next_disturbance_update_time_) {
        sampleDisturbanceForce();
    } else {
        double* appliedWrench = data_->xfrc_applied + 6 * disturbance_body_id_;
        for (int i = 0; i < 6; ++i) {
            appliedWrench[i] = 0.0;
        }
    }
}

void MujocoSimulation::render() {
    // Check if window exists
    if (window_) {
        glfwMakeContextCurrent(window_);
        mjr_setBuffer(mjFB_WINDOW, &context_);
        if (context_.currentBuffer != mjFB_WINDOW) {
            static bool warned_window_buffer = false;
            if (!warned_window_buffer) {
                RCLCPP_ERROR(node_->get_logger(),
                             "MuJoCo renderer is not bound to the window framebuffer. Viewer output will stay black.");
                warned_window_buffer = true;
            }
            return;
        }

        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);

        if (width <= 0 || height <= 0) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid framebuffer size: %d x %d", width, height);
            return;
        }

        mjrRect viewport = {0, 0, width, height};
        
        // Update the scene
        if (!model_ || !data_) {
            RCLCPP_ERROR(node_->get_logger(), "Model or data is null.");
            return;
        }

        mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
        appendDisturbanceArrowToScene();

        // Render the scene
        mjr_render(viewport, &scene_, &context_);
        renderDisturbanceOverlay(viewport);

        // Swap buffers to display rendered image
        glfwSwapBuffers(window_);

        // Poll for and process events (e.g., keyboard input, window resizing)
        glfwPollEvents();

        ++rendered_frame_count_;
        const auto now = std::chrono::steady_clock::now();
        const std::chrono::duration<double> elapsed = now - render_fps_window_start_;
        if (elapsed.count() >= 0.5) {
            measured_render_fps_ = static_cast<double>(rendered_frame_count_) / elapsed.count();
            const double current_sim_time = data_ ? data_->time : 0.0;
            measured_real_time_factor_ =
                (current_sim_time - render_rtf_window_start_sim_time_) / elapsed.count();
            rendered_frame_count_ = 0;
            render_fps_window_start_ = now;
            render_rtf_window_start_sim_time_ = current_sim_time;
        }
    }
}

double MujocoSimulation::getCurrentDisturbanceForceMagnitude() const {
    if (data_ == nullptr || disturbance_body_id_ < 0) {
        return 0.0;
    }

    const double* appliedWrench = data_->xfrc_applied + 6 * disturbance_body_id_;
    return std::sqrt(appliedWrench[0] * appliedWrench[0] +
                     appliedWrench[1] * appliedWrench[1] +
                     appliedWrench[2] * appliedWrench[2]);
}

double MujocoSimulation::getLastDisturbanceForceMagnitude() const {
    return std::sqrt(last_disturbance_wrench_[0] * last_disturbance_wrench_[0] +
                     last_disturbance_wrench_[1] * last_disturbance_wrench_[1] +
                     last_disturbance_wrench_[2] * last_disturbance_wrench_[2]);
}

void MujocoSimulation::renderDisturbanceOverlay(const mjrRect& viewport) {
    char fpsText[64];
    std::snprintf(fpsText, sizeof(fpsText), "FPS %.1f / %.1f\nRTF %.2f",
                  measured_render_fps_, render_frequency_, measured_real_time_factor_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, fpsText, nullptr, &context_);

    if (disturbance_enabled_ && disturbance_body_id_ >= 0) {
        char overlayText[256];
        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;
        const double currentForceMagnitude = getCurrentDisturbanceForceMagnitude();
        const bool forceActive = currentForceMagnitude > 1e-6;
        if (forceActive && data_ != nullptr && disturbance_body_id_ >= 0) {
            const double* appliedWrench = data_->xfrc_applied + 6 * disturbance_body_id_;
            fx = appliedWrench[0];
            fy = appliedWrench[1];
            fz = appliedWrench[2];
        } else {
            fx = last_disturbance_wrench_[0];
            fy = last_disturbance_wrench_[1];
            fz = last_disturbance_wrench_[2];
        }

        const double forceMagnitude = forceActive ? currentForceMagnitude : getLastDisturbanceForceMagnitude();
        std::snprintf(overlayText, sizeof(overlayText),
                      "Disturbance: ON\nTarget: %s\nScale: %.2f\nLast force: %.1f N\nFx: %.1f   Fy: %.1f   Fz: %.1f\nKeys: 0 trunk, 1 LF, 2 RF, 3 LH, 4 RH",
                      getDisturbanceBodyLabel(), getCurrentDisturbanceForceScale(), forceMagnitude, fx, fy, fz);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, overlayText, nullptr, &context_);
    }
}

void MujocoSimulation::appendDisturbanceArrowToScene() {
    if (!disturbance_enabled_ || data_ == nullptr || disturbance_body_id_ < 0 || disturbance_arrow_scale_ <= 0.0) {
        return;
    }

    const double fx = current_disturbance_wrench_[0];
    const double fy = current_disturbance_wrench_[1];
    const double fz = current_disturbance_wrench_[2];
    const double forceNorm = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (forceNorm < 1e-6 || scene_.ngeom >= scene_.maxgeom) {
        return;
    }

    constexpr mjtNum kArrowHeightOffset = 0.12;
    const mjtNum from[3] = {
        data_->xpos[3 * disturbance_body_id_ + 0],
        data_->xpos[3 * disturbance_body_id_ + 1],
        data_->xpos[3 * disturbance_body_id_ + 2] + kArrowHeightOffset
    };
    const mjtNum to[3] = {
        from[0] + disturbance_arrow_scale_ * fx,
        from[1] + disturbance_arrow_scale_ * fy,
        from[2] + disturbance_arrow_scale_ * fz
    };
    const float rgba[4] = {1.0f, 0.2f, 0.2f, 0.6f};
    mjvGeom* disturbanceGeom = &scene_.geoms[scene_.ngeom];
    mjv_initGeom(disturbanceGeom, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba);
    mjv_connector(disturbanceGeom, mjGEOM_ARROW, 0.02, from, to);
    disturbanceGeom->category = mjCAT_DECOR;
    disturbanceGeom->segid = -1;
    disturbanceGeom->objid = -1;
    disturbanceGeom->transparent = 1;
    disturbanceGeom->emission = 1.0f;
    scene_.ngeom++;
}

void MujocoSimulation::clearActuatorCommandState() {
    for (int i = 0; i < 12; ++i) {
        Joint_position_buffer_[i] = 0.0;
        Joint_velocity_buffer_[i] = 0.0;
        Joint_torque_buffer_[i] = 0.0;
        Joint_position_[i] = 0.0;
        Joint_velocity_[i] = 0.0;
        Joint_torque_[i] = 0.0;
    }

    actuator_mode_ = 2;

    if (data_ != nullptr) {
        std::fill(data_->ctrl, data_->ctrl + model_->nu, 0.0);
    }
}

void MujocoSimulation::resetRobotPose() {
    if (model_ == nullptr || data_ == nullptr) {
        return;
    }

    clearDisturbanceForce();
    disturbance_enabled_ = false;

    if (static_cast<int>(initial_qpos_.size()) == model_->nq) {
        std::copy(initial_qpos_.begin(), initial_qpos_.end(), data_->qpos);
    } else if (model_->nq >= 7) {
        data_->qpos[0] = 0.0;
        data_->qpos[1] = 0.0;
        data_->qpos[2] = 0.30;
        data_->qpos[3] = initial_base_quat_[0];
        data_->qpos[4] = initial_base_quat_[1];
        data_->qpos[5] = initial_base_quat_[2];
        data_->qpos[6] = initial_base_quat_[3];
    }

    if (static_cast<int>(initial_qvel_.size()) == model_->nv) {
        std::copy(initial_qvel_.begin(), initial_qvel_.end(), data_->qvel);
    } else {
        const int baseVelocityDim = std::min(model_->nv, 6);
        for (int i = 0; i < baseVelocityDim; ++i) {
            data_->qvel[i] = 0.0;
        }
    }

    clearActuatorCommandState();
    mj_forward(model_, data_);
}

void MujocoSimulation::emergencyOverrideStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    (void)msg;
}


void MujocoSimulation::control_callback(legged_msgs::msg::JointControlData::SharedPtr msg)
{
    applyJointControl(*msg);
    publish_state_data();
    publish_sensor_data();
}

void MujocoSimulation::applyJointControl(const legged_msgs::msg::JointControlData& msg)
{
    const bool valid_position_size = msg.joint_position.size() == 12;
    const bool valid_velocity_size = msg.joint_velocity.size() == 12;
    const bool valid_torque_size = msg.joint_torque.size() == 12;
    if (!valid_position_size || !valid_velocity_size || !valid_torque_size) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Invalid joint command sizes. position=%zu velocity=%zu torque=%zu (expected 12 each). Ignoring command.",
                     msg.joint_position.size(), msg.joint_velocity.size(), msg.joint_torque.size());
        return;
    }

    Start_simulate_ = false;
    Start_control_ = true;
    actuator_mode_ = static_cast<int>(msg.actuator_mode);

    for (size_t i = 0; i < msg.joint_position.size(); ++i) {
        Joint_position_[i] = msg.joint_position[i];
        Joint_velocity_[i] = msg.joint_velocity[i];
        Joint_torque_[i] = msg.joint_torque[i];
    }
}


bool MujocoSimulation::checkCollision(const mjData* d, int geom1_id, int geom2_id) {
    for (int i = 0; i < d->ncon; ++i) {
        const mjContact& contact = d->contact[i];
        // std::cout << "Contact Info:\n";
        // std::cout << "  Geom1: " << contact.geom1 << "\n";
        // std::cout << "  Geom2: " << contact.geom2 << "\n";
        // std::cout << "  Distance: " << contact.dist << "\n";
        if ((contact.geom1 == geom1_id && contact.geom2 == geom2_id) ||
            (contact.geom1 == geom2_id && contact.geom2 == geom1_id)) {
            // std::cout << "Collision detected between geom " 
            //           << geom1_id << " and geom " << geom2_id 
            //           << ". Distance: " << contact.dist << std::endl;
            return 1;
        }
    }
    return 0;
}

void MujocoSimulation::populate_state_message(legged_msgs::msg::SimulatorStateData& state)
{
    state.simulation_time = data_->time;

    state.base_quat_values.clear();
    for (int i = 0; i < 4; ++i) {
        state.base_quat_values.push_back(static_cast<double>(data_->sensordata[i + 34]));
    }

    state.base_pose_values.clear();
    state.base_angvel_values.clear();
    state.base_linvel_values.clear();
    for (int i = 0; i < 3; ++i) {
        state.base_pose_values.push_back(static_cast<double>(data_->sensordata[i + 38]));
        state.base_angvel_values.push_back(static_cast<double>(data_->sensordata[i + 41]));
        state.base_linvel_values.push_back(static_cast<double>(data_->sensordata[i + 44]));
    }

    state.joint_position_values.clear();
    state.joint_velocity_values.clear();
    state.joint_torque_values.clear();
    for (int i = 0; i < 12; ++i) {
        state.joint_position_values.push_back(static_cast<double>(data_->sensordata[i + 10]));
        state.joint_velocity_values.push_back(static_cast<double>(data_->sensordata[i + 22]));
        state.joint_torque_values.push_back(static_cast<double>(data_->actuator_force[i]));
    }

    state.contact_flags.resize(geom_ids_.size() - 1);
    for (size_t i = 0; i < (geom_ids_.size() - 1); ++i) {
        state.contact_flags[i] = checkCollision(data_, geom_ids_[0], geom_ids_[i + 1]);
    }
}


void MujocoSimulation::publish_state_data()
{
    auto message = legged_msgs::msg::SimulatorStateData();
    populate_state_message(message);

    if (!state_pub_) {
        return;
    }

    RCLCPP_INFO(node_->get_logger(),
            "Simulation time = [%f], \n"
            "Publishing base quat data: [%f, %f, %f, %f], \n"
            "Publishing base pos data: [%f, %f, %f], \n"
            "Publishing base angvel data: [%f, %f, %f], \n"
            "Publishing base linvel data: [%f, %f, %f], \n"
            "Publishing joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Publishing joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Publishing joint torque data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Publishing contact flag: [%zu, %zu, %zu, %zu]",
            message.simulation_time,
            message.base_quat_values[0], message.base_quat_values[1], 
            message.base_quat_values[2],message.base_quat_values[3],
            message.base_pose_values[0], message.base_pose_values[1],message.base_pose_values[2], 
            message.base_angvel_values[0], message.base_angvel_values[1],message.base_angvel_values[2], 
            message.base_linvel_values[0], message.base_linvel_values[1],message.base_linvel_values[2],
            message.joint_position_values[0], message.joint_position_values[1],
            message.joint_position_values[2], message.joint_position_values[3],
            message.joint_position_values[4], message.joint_position_values[5],
            message.joint_position_values[6], message.joint_position_values[7],
            message.joint_position_values[8], message.joint_position_values[9],
            message.joint_position_values[10], message.joint_position_values[11],
            message.joint_velocity_values[0], message.joint_velocity_values[1],
            message.joint_velocity_values[2], message.joint_velocity_values[3],
            message.joint_velocity_values[4], message.joint_velocity_values[5],
            message.joint_velocity_values[6], message.joint_velocity_values[7],
            message.joint_velocity_values[8], message.joint_velocity_values[9],
            message.joint_velocity_values[10], message.joint_velocity_values[11],
            message.joint_torque_values[0], message.joint_torque_values[1],
            message.joint_torque_values[2], message.joint_torque_values[3],
            message.joint_torque_values[4], message.joint_torque_values[5],
            message.joint_torque_values[6], message.joint_torque_values[7],
            message.joint_torque_values[8], message.joint_torque_values[9],
            message.joint_torque_values[10], message.joint_torque_values[11],
            static_cast<size_t>(message.contact_flags[0]), static_cast<size_t>(message.contact_flags[1]),
            static_cast<size_t>(message.contact_flags[2]), static_cast<size_t>(message.contact_flags[3]));
    RCLCPP_INFO(node_->get_logger(),
            "Simulation time = [%f]",message.simulation_time);

    state_pub_->publish(message);
}


void MujocoSimulation::populate_sensor_message(legged_msgs::msg::SimulatorSensorData& message)
{
    message.simulation_time = data_->time;

    message.imu_quat_values.clear();
    message.imu_angvel_values.clear();
    message.imu_linacc_values.clear();
    for (int i = 0; i < 4; ++i) {
        message.imu_quat_values.push_back(static_cast<double>(data_->sensordata[i]));
    }
    for (int i = 0; i < 3; ++i) {
        message.imu_angvel_values.push_back(static_cast<double>(data_->sensordata[i + 4]));
        message.imu_linacc_values.push_back(static_cast<double>(data_->sensordata[i + 7]));
    }

    message.joint_position_values.clear();
    message.joint_velocity_values.clear();
    for (int i = 0; i < 12; ++i) {
        message.joint_position_values.push_back(static_cast<double>(data_->sensordata[i + 10]));
        message.joint_velocity_values.push_back(static_cast<double>(data_->sensordata[i + 22]));
    }

    message.contact_flags.resize(geom_ids_.size() - 1);
    for (size_t i = 0; i < (geom_ids_.size() - 1); ++i) {
        message.contact_flags[i] = checkCollision(data_, geom_ids_[0], geom_ids_[i + 1]);
    }
}

void MujocoSimulation::publish_sensor_data()
{
    auto message = legged_msgs::msg::SimulatorSensorData();
    populate_sensor_message(message);

    if (!sensor_pub_) {
        return;
    }

    // RCLCPP_INFO(node_->get_logger(),
    //         "Simulation time = [%f], \n"
    //         "Publishing IMU quat data: [%f, %f, %f, %f], \n"
    //         "Publishing IMU angvel data: [%f, %f, %f], \n"
    //         "Publishing IMU linacc data: [%f, %f, %f], \n"
    //         "Publishing joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
    //         "Publishing joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
    //         "Publishing contact flag: [%zu, %zu, %zu, %zu]",
    //         message.simulation_time,
    //         message.imu_quat_values[0], message.imu_quat_values[1], message.imu_quat_values[2],
    //         message.imu_quat_values[3],
    //         message.imu_angvel_values[0], message.imu_angvel_values[1], message.imu_angvel_values[2],
    //         message.imu_linacc_values[0], message.imu_linacc_values[1], message.imu_linacc_values[2],
    //         message.joint_position_values[0], message.joint_position_values[1],
    //         message.joint_position_values[2], message.joint_position_values[3],
    //         message.joint_position_values[4], message.joint_position_values[5],
    //         message.joint_position_values[6], message.joint_position_values[7],
    //         message.joint_position_values[8], message.joint_position_values[9],
    //         message.joint_position_values[10], message.joint_position_values[11],
    //         message.joint_velocity_values[0], message.joint_velocity_values[1],
    //         message.joint_velocity_values[2], message.joint_velocity_values[3],
    //         message.joint_velocity_values[4], message.joint_velocity_values[5],
    //         message.joint_velocity_values[6], message.joint_velocity_values[7],
    //         message.joint_velocity_values[8], message.joint_velocity_values[9],
    //         message.joint_velocity_values[10], message.joint_velocity_values[11],
    //         static_cast<size_t>(message.contact_flags[0]), static_cast<size_t>(message.contact_flags[1]),
    //         static_cast<size_t>(message.contact_flags[2]), static_cast<size_t>(message.contact_flags[3]));

    sensor_pub_->publish(message);
}

void MujocoSimulation::stepControlPeriod()
{
    const mjtNum simstart = data_->time;
    while (data_->time - simstart < 1.0 / control_frequency_) {
        simulateStep();
    }
}


void MujocoSimulation::start_control_service(const std::shared_ptr<legged_msgs::srv::StartControl::Request> request,
    std::shared_ptr<legged_msgs::srv::StartControl::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Received: %s", request->start ? "true" : "false");

    //stop simulate and wait for control message
    Start_simulate_=false;

    legged_msgs::msg::SimulatorStateData state;
    populate_state_message(state);

    // response
    response->state = state;
    response->success = true;

    RCLCPP_INFO(node_->get_logger(),
            "Simulation time = [%f], \n"
            "Responding base quat data: [%f, %f, %f, %f], \n"
            "Responding base pos data: [%f, %f, %f], \n"
            "Responding base angvel data: [%f, %f, %f], \n"
            "Responding base linvel data: [%f, %f, %f], \n"
            "Responding joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Responding joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Responding joint torque data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
            "Responding contact flag: [%zu, %zu, %zu, %zu]",
            response->state.simulation_time,
            response->state.base_quat_values[0], response->state.base_quat_values[1], 
            response->state.base_quat_values[2],response->state.base_quat_values[3],
            response->state.base_pose_values[0], response->state.base_pose_values[1],response->state.base_pose_values[2], 
            response->state.base_angvel_values[0], response->state.base_angvel_values[1],response->state.base_angvel_values[2], 
            response->state.base_linvel_values[0], response->state.base_linvel_values[1],response->state.base_linvel_values[2],
            response->state.joint_position_values[0], response->state.joint_position_values[1],
            response->state.joint_position_values[2], response->state.joint_position_values[3],
            response->state.joint_position_values[4], response->state.joint_position_values[5],
            response->state.joint_position_values[6], response->state.joint_position_values[7],
            response->state.joint_position_values[8], response->state.joint_position_values[9],
            response->state.joint_position_values[10], response->state.joint_position_values[11],
            response->state.joint_velocity_values[0], response->state.joint_velocity_values[1],
            response->state.joint_velocity_values[2], response->state.joint_velocity_values[3],
            response->state.joint_velocity_values[4], response->state.joint_velocity_values[5],
            response->state.joint_velocity_values[6], response->state.joint_velocity_values[7],
            response->state.joint_velocity_values[8], response->state.joint_velocity_values[9],
            response->state.joint_velocity_values[10], response->state.joint_velocity_values[11],
            response->state.joint_torque_values[0], response->state.joint_torque_values[1],
            response->state.joint_torque_values[2], response->state.joint_torque_values[3],
            response->state.joint_torque_values[4], response->state.joint_torque_values[5],
            response->state.joint_torque_values[6], response->state.joint_torque_values[7],
            response->state.joint_torque_values[8], response->state.joint_torque_values[9],
            response->state.joint_torque_values[10], response->state.joint_torque_values[11],
            static_cast<size_t>(response->state.contact_flags[0]), static_cast<size_t>(response->state.contact_flags[1]),
            static_cast<size_t>(response->state.contact_flags[2]), static_cast<size_t>(response->state.contact_flags[3]));
}
