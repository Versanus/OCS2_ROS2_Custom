#ifndef MUJOCO_SIMULATION_HPP
#define MUJOCO_SIMULATION_HPP

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <GLFW/glfw3.h>  // For rendering and window handling
#include <array>
#include <chrono>
#include <mutex>
#include <random>
#include <string>
#include <vector>
#include <ocs2_core/misc/LoadData.h>
//#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "legged_msgs/msg/simulator_state_data.hpp"
#include "legged_msgs/msg/simulator_sensor_data.hpp"
#include "legged_msgs/msg/joint_control_data.hpp"
#include "legged_msgs/srv/start_control.hpp"

class MujocoSimulation
{
public:
    struct RuntimeOptions {
        double timestep = 0.0;
        double controlFrequency = 0.0;
        double baseKp = 0.0;
        double baseKd = 0.0;
        bool directPositionControl = false;
        RuntimeOptions() = default;
    };

    MujocoSimulation(const rclcpp::Node::SharedPtr& node,
    const std::string& xmlFile,
    const std::string& simulatorFile,
    bool exposeRosInterface = true);
    MujocoSimulation(const rclcpp::Node::SharedPtr& node,
    const std::string& xmlFile,
    const std::string& simulatorFile,
    bool exposeRosInterface,
    const RuntimeOptions& runtimeOptions);
    ~MujocoSimulation();
    
    mjData* getData() { return data_; }
    bool* getSimuState() { return &Start_simulate_; }
    bool* getContrlState() { return &Start_control_; }
    void loadModel(const std::string& modelPath, const std::string& configPath, const RuntimeOptions& runtimeOptions);
    //void copyControlFromBuffer();
    void simulateStep();
    void renderSetting(const std::string& configFile);
    void render();
    void control_callback(const legged_msgs::msg::JointControlData::SharedPtr msg);
    void applyJointControl(const legged_msgs::msg::JointControlData& msg);
    bool checkCollision(const mjData* d, int geom1_id, int geom2_id);
    void run();
    void publish_state_data();
    void publish_sensor_data();
    void populate_state_message(legged_msgs::msg::SimulatorStateData& state);
    void populate_sensor_message(legged_msgs::msg::SimulatorSensorData& sensor);
    void start_control_service(const std::shared_ptr<legged_msgs::srv::StartControl::Request> request,
        std::shared_ptr<legged_msgs::srv::StartControl::Response> response);
    void stepControlPeriod();
    void updateDisturbanceForce();
    void clearDisturbanceForce();
    void scheduleNextDisturbance();
    void sampleDisturbanceForce();
    void appendDisturbanceArrowToScene();
    void renderDisturbanceOverlay(const mjrRect& viewport);
    bool setDisturbanceBody(const char* bodyName, const char* bodyLabel);
    const char* getDisturbanceBodyLabel() const;
    double getCurrentDisturbanceForceScale() const;
    double getCurrentDisturbanceForceMagnitude() const;
    double getLastDisturbanceForceMagnitude() const;
    void resetRobotPose();
    void clearActuatorCommandState();
    void latchDirectPositionTargetsFromCurrentState();
    void emergencyOverrideStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
    double getControlFrequency() const { return control_frequency_; }
    double getRenderFrequency() const { return render_frequency_; }
    double getTimestep() const { return timestep_; }
    double getBaseKp() const { return baseKp_; }
    double getBaseKd() const { return baseKd_; }
    bool usesDirectPositionControl() const { return directPositionControl_; }
    bool exposesRosInterface() const { return exposeRosInterface_; }

    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);

    

private:
    static MujocoSimulation* instance_; // 静态实例指针
    
    rclcpp::Node::SharedPtr node_;
    mjModel* model_;
    mjData* data_;
    mjvScene scene_;
    mjvCamera cam_;
    mjvOption opt_;
    mjrContext context_;
    bool exposeRosInterface_;

    // rendering
    bool render_appearance_;
    bool render_inertia_;
    bool visualize_contacts_;
    bool tracking_base_;
    double render_frequency_ = 60.0;
    double measured_render_fps_;
    double measured_real_time_factor_;
    std::size_t rendered_frame_count_ = 0;
    std::chrono::steady_clock::time_point render_fps_window_start_ = std::chrono::steady_clock::now();
    double render_rtf_window_start_sim_time_;

    // GLFW window
    GLFWwindow* window_;

    // ros2
    rclcpp::Publisher<legged_msgs::msg::SimulatorStateData>::SharedPtr state_pub_; //real state
    rclcpp::Publisher<legged_msgs::msg::SimulatorSensorData>::SharedPtr sensor_pub_; // sensor date
    rclcpp::Subscription<legged_msgs::msg::JointControlData>::SharedPtr joint_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr emergency_override_state_sub_;
    rclcpp::Service<legged_msgs::srv::StartControl>::SharedPtr start_control_server_;

    // motor control
    double Joint_position_buffer_[12];
    double Joint_velocity_buffer_[12];
    double Joint_torque_buffer_[12];
    double Joint_position_[12];
    double Joint_velocity_[12];
    double Joint_torque_[12];
    double timestep_;
    double control_frequency_;
    double baseKp_ = 10.0;
    double baseKd_ = 0.30;
    bool directPositionControl_ = false;
    std::array<int, 12> actuator_index_by_joint_{};
    std::array<int, 12> joint_qpos_address_by_joint_{};
    std::array<int, 12> joint_qvel_address_by_joint_{};
    double kpRatio_;
    double kdRatio_;
    bool Start_control_=false;
    bool Start_simulate_=false;
    std::array<double, 12> last_logged_joint_position_command_{};
    bool has_logged_joint_position_command_ = false;

    // random base disturbance
    bool disturbance_enabled_ = false;
    int disturbance_body_id_ = -1;
    int disturbance_body_selection_index_ = 0;
    std::string disturbance_body_label_ = "trunk";
    std::array<double, 5> disturbance_force_scales_{{1.0, 1.0, 1.0, 1.0, 1.0}};
    double disturbance_force_min_ = 20.0;
    double disturbance_force_max_ = 60.0;
    double disturbance_vertical_scale_ = 0.35;
    double disturbance_interval_min_ = 0.50;
    double disturbance_interval_max_ = 1.20;
    double disturbance_impulse_duration_ = 0.05;
    double disturbance_arrow_scale_ = 0.005;
    double next_disturbance_update_time_;
    double disturbance_active_until_time_;
    std::array<double, 6> current_disturbance_wrench_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> last_disturbance_wrench_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::mt19937 disturbance_rng_;
    std::array<double, 4> initial_base_quat_{{1.0, 0.0, 0.0, 0.0}};
    std::vector<double> initial_qpos_;
    std::vector<double> initial_qvel_;

    // contact flag
    std::vector<int> geom_ids_; // floor, LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT
    //int geom_id_[5]; // floor, LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT

    static bool button_left, button_middle, button_right;
    static double lastx, lasty;
};

#endif  // MUJOCO_SIMULATION_H
