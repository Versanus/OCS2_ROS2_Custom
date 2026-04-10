#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stm2ros/msg/joint_control_state.hpp>

namespace {

constexpr std::size_t kJointCmdSize = 12;

struct ActiveJoint {
  std::string name;
  std::string model_name;
  int q_index = 0;
  int q_size = 0;
  int v_index = 0;
  std::size_t output_index = 0;
  double lower_position_limit = 0.0;
  double upper_position_limit = 0.0;
  double effort_limit = 0.0;
  double velocity_limit = 0.0;
};

double clamp(double value, double lower, double upper) {
  return std::min(std::max(value, lower), upper);
}

}  // namespace

class OneLegInverseDynamicsNode final : public rclcpp::Node {
 public:
  OneLegInverseDynamicsNode() : Node("one_leg_inverse_dynamics_node") {
    declareParameters();
    loadParameters();
    loadPinocchioModel();
    validateJointLayout();
    validateDesiredPositions(desired_positions_, model_position_offsets_);

    if (enabled_) {
      enabled_since_ = now();
    }

    q_measured_ = pinocchio::neutral(model_);
    v_measured_ = Eigen::VectorXd::Zero(model_.nv);
    measured_hardware_positions_.assign(active_joint_names_.size(), 0.0);
    latest_output_positions_.assign(kJointCmdSize, 0.0);
    last_command_torque_.assign(kJointCmdSize, 0.0);

    command_pub_ = create_publisher<stm2ros::msg::JointControlState>(command_topic_, 10);
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        state_topic_, 10, std::bind(&OneLegInverseDynamicsNode::stateCallback, this, std::placeholders::_1));

    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&OneLegInverseDynamicsNode::setParametersCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&OneLegInverseDynamicsNode::controlTimerCallback, this));

    RCLCPP_WARN(get_logger(),
                "One-leg inverse dynamics node is running in isolated joint_cmd mode. "
                "It starts disabled; set parameter 'enabled:=true' only after verifying state names.");
  }

 private:
  void declareParameters() {
    declare_parameter<std::string>("urdf_path", "");
    declare_parameter<std::string>("state_topic", "htdw_joint_state");
    declare_parameter<std::string>("command_topic", "joint_cmd");
    declare_parameter<double>("control_rate_hz", 100.0);
    declare_parameter<double>("max_state_age_sec", 0.15);
    declare_parameter<bool>("enabled", false);
    declare_parameter<bool>("soft_stop", false);
    declare_parameter<bool>("clear_faults", false);
    declare_parameter<bool>("feedforward_torque_enabled", false);
    declare_parameter<std::vector<double>>("joint_feedforward_enabled", {0.0, 0.0, 0.0});

    declare_parameter<std::vector<std::string>>("active_joint_names", {"FL_HipX", "FL_HipY", "FL_Knee"});
    declare_parameter<std::vector<std::string>>("model_joint_names",
                                                {"fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint"});
    declare_parameter<std::vector<std::string>>(
        "output_joint_names", {"FL_HipX", "FL_HipY", "FL_Knee", "FR_HipX", "FR_HipY", "FR_Knee",
                               "HL_HipX", "HL_HipY", "HL_Knee", "HR_HipX", "HR_HipY", "HR_Knee"});

    declare_parameter<std::vector<double>>("desired_positions", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("desired_velocities", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("desired_accelerations", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("model_position_offsets", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("id_kp", {1.5, 1.5, 1.0});
    declare_parameter<std::vector<double>>("id_kd", {0.08, 0.08, 0.05});
    declare_parameter<std::vector<double>>("max_torque", {6.0, 6.0, 6.0});
    declare_parameter<std::vector<double>>("max_torque_rate", {15.0, 15.0, 15.0});
    declare_parameter<std::vector<double>>("max_position_error", {0.8, 0.8, 0.8});
    declare_parameter<std::vector<double>>("max_velocity", {15.0, 15.0, 15.0});
    declare_parameter<double>("startup_ramp_sec", 2.0);
    declare_parameter<double>("command_kp", 0.0);
    declare_parameter<double>("command_kd", 0.0);
  }

  void loadParameters() {
    urdf_path_ = get_parameter("urdf_path").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    command_topic_ = get_parameter("command_topic").as_string();
    control_rate_hz_ = std::max(1.0, get_parameter("control_rate_hz").as_double());
    max_state_age_sec_ = std::max(0.001, get_parameter("max_state_age_sec").as_double());
    enabled_ = get_parameter("enabled").as_bool();
    soft_stop_ = get_parameter("soft_stop").as_bool();
    feedforward_torque_enabled_ = get_parameter("feedforward_torque_enabled").as_bool();

    active_joint_names_ = get_parameter("active_joint_names").as_string_array();
    model_joint_names_ = get_parameter("model_joint_names").as_string_array();
    output_joint_names_ = get_parameter("output_joint_names").as_string_array();
    if (active_joint_names_.empty()) {
      throw std::runtime_error("active_joint_names must not be empty.");
    }
    if (model_joint_names_.empty()) {
      model_joint_names_ = active_joint_names_;
    }
    if (model_joint_names_.size() != active_joint_names_.size()) {
      throw std::runtime_error("model_joint_names must be empty or match active_joint_names length.");
    }
    if (output_joint_names_.size() != kJointCmdSize) {
      throw std::runtime_error("output_joint_names must contain exactly 12 joints for stm2ros/JointControlState.");
    }

    desired_positions_ = expandVector(get_parameter("desired_positions").as_double_array(), 0.0, "desired_positions");
    desired_velocities_ = expandVector(get_parameter("desired_velocities").as_double_array(), 0.0, "desired_velocities");
    desired_accelerations_ =
        expandVector(get_parameter("desired_accelerations").as_double_array(), 0.0, "desired_accelerations");
    model_position_offsets_ =
        expandVector(get_parameter("model_position_offsets").as_double_array(), 0.0, "model_position_offsets");
    joint_feedforward_enabled_ =
        expandVector(get_parameter("joint_feedforward_enabled").as_double_array(), 0.0, "joint_feedforward_enabled");
    id_kp_ = expandVector(get_parameter("id_kp").as_double_array(), 0.0, "id_kp");
    id_kd_ = expandVector(get_parameter("id_kd").as_double_array(), 0.0, "id_kd");
    max_torque_ = expandVector(get_parameter("max_torque").as_double_array(), 0.0, "max_torque");
    max_torque_rate_ = expandVector(get_parameter("max_torque_rate").as_double_array(), 0.0, "max_torque_rate");
    max_position_error_ =
        expandVector(get_parameter("max_position_error").as_double_array(), 0.0, "max_position_error");
    max_velocity_ = expandVector(get_parameter("max_velocity").as_double_array(), 0.0, "max_velocity");
    requireFiniteVector(desired_positions_, "desired_positions");
    requireFiniteVector(desired_velocities_, "desired_velocities");
    requireFiniteVector(desired_accelerations_, "desired_accelerations");
    requireFiniteVector(model_position_offsets_, "model_position_offsets");
    requireUnitIntervalVector(joint_feedforward_enabled_, "joint_feedforward_enabled");
    requireNonnegativeVector(id_kp_, "id_kp");
    requireNonnegativeVector(id_kd_, "id_kd");
    requireNonnegativeVector(max_torque_, "max_torque");
    requireNonnegativeVector(max_torque_rate_, "max_torque_rate");
    requireNonnegativeVector(max_position_error_, "max_position_error");
    requireNonnegativeVector(max_velocity_, "max_velocity");
    startup_ramp_sec_ = requireNonnegativeDouble(get_parameter("startup_ramp_sec").as_double(), "startup_ramp_sec");
    command_kp_ = requireNonnegativeDouble(get_parameter("command_kp").as_double(), "command_kp");
    command_kd_ = requireNonnegativeDouble(get_parameter("command_kd").as_double(), "command_kd");
  }

  std::vector<double> expandVector(const std::vector<double>& values, double default_value, const std::string& name) const {
    if (values.empty()) {
      return std::vector<double>(active_joint_names_.size(), default_value);
    }
    if (values.size() == 1) {
      return std::vector<double>(active_joint_names_.size(), values.front());
    }
    if (values.size() == active_joint_names_.size()) {
      return values;
    }
    throw std::runtime_error(name + " must be empty, scalar, or match active_joint_names length.");
  }

  void requireFiniteVector(const std::vector<double>& values, const std::string& name) const {
    for (const double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error(name + " contains a non-finite value.");
      }
    }
  }

  void requireNonnegativeVector(const std::vector<double>& values, const std::string& name) const {
    requireFiniteVector(values, name);
    for (const double value : values) {
      if (value < 0.0) {
        throw std::runtime_error(name + " must not contain negative values.");
      }
    }
  }

  double requireNonnegativeDouble(double value, const std::string& name) const {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::runtime_error(name + " must be a finite nonnegative value.");
    }
    return value;
  }

  void requireUnitIntervalVector(const std::vector<double>& values, const std::string& name) const {
    requireFiniteVector(values, name);
    for (const double value : values) {
      if (value < 0.0 || value > 1.0) {
        throw std::runtime_error(name + " values must be between 0.0 and 1.0.");
      }
    }
  }

  void loadPinocchioModel() {
    if (urdf_path_.empty()) {
      throw std::runtime_error("urdf_path parameter is empty.");
    }

    pinocchio::urdf::buildModel(urdf_path_, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    RCLCPP_INFO(get_logger(), "Loaded URDF for Pinocchio: %s (nq=%d, nv=%d)", urdf_path_.c_str(), model_.nq, model_.nv);
  }

  void validateJointLayout() {
    active_joints_.clear();
    for (std::size_t i = 0; i < active_joint_names_.size(); ++i) {
      const auto& joint_name = active_joint_names_[i];
      const auto& model_joint_name = model_joint_names_[i];
      const auto joint_id = model_.getJointId(model_joint_name);
      if (joint_id >= static_cast<pinocchio::JointIndex>(model_.njoints)) {
        throw std::runtime_error("Model joint '" + model_joint_name + "' for active joint '" + joint_name +
                                 "' does not exist in URDF.");
      }
      if ((model_.nqs[joint_id] != 1 && model_.nqs[joint_id] != 2) || model_.nvs[joint_id] != 1) {
        throw std::runtime_error("Model joint '" + model_joint_name +
                                 "' must be a single-DoF revolute or continuous joint.");
      }

      const auto output_it = std::find(output_joint_names_.begin(), output_joint_names_.end(), joint_name);
      if (output_it == output_joint_names_.end()) {
        throw std::runtime_error("Active joint '" + joint_name + "' does not exist in output_joint_names.");
      }

      ActiveJoint joint;
      joint.name = joint_name;
      joint.model_name = model_joint_name;
      joint.q_index = model_.idx_qs[joint_id];
      joint.q_size = model_.nqs[joint_id];
      joint.v_index = model_.idx_vs[joint_id];
      joint.output_index = static_cast<std::size_t>(std::distance(output_joint_names_.begin(), output_it));
      if (joint.q_size == 1) {
        joint.lower_position_limit = model_.lowerPositionLimit[joint.q_index];
        joint.upper_position_limit = model_.upperPositionLimit[joint.q_index];
      } else {
        joint.lower_position_limit = -std::numeric_limits<double>::infinity();
        joint.upper_position_limit = std::numeric_limits<double>::infinity();
      }
      joint.effort_limit = std::abs(model_.effortLimit[joint.v_index]);
      joint.velocity_limit = std::abs(model_.velocityLimit[joint.v_index]);
      active_joints_.push_back(joint);
    }
  }

  double modelPositionFromHardware(std::size_t active_joint_index, double hardware_position) const {
    return hardware_position + model_position_offsets_[active_joint_index];
  }

  void writeModelPosition(Eigen::VectorXd& q, const ActiveJoint& joint, double model_position) const {
    if (joint.q_size == 1) {
      q[joint.q_index] = model_position;
    } else {
      q[joint.q_index] = std::cos(model_position);
      q[joint.q_index + 1] = std::sin(model_position);
    }
  }

  void validateDesiredPositions(const std::vector<double>& desired_positions,
                                const std::vector<double>& model_position_offsets) const {
    for (std::size_t i = 0; i < active_joints_.size(); ++i) {
      const auto& joint = active_joints_[i];
      const double desired_position = desired_positions[i];
      if (!std::isfinite(desired_position)) {
        throw std::runtime_error("desired_positions contains a non-finite value for " + joint.name + ".");
      }

      const bool has_finite_limits = std::isfinite(joint.lower_position_limit) &&
                                     std::isfinite(joint.upper_position_limit) &&
                                     joint.lower_position_limit < joint.upper_position_limit;
      const double model_desired_position = desired_position + model_position_offsets[i];
      if (has_finite_limits &&
          (model_desired_position < joint.lower_position_limit || model_desired_position > joint.upper_position_limit)) {
        throw std::runtime_error("desired position for " + joint.name + " is outside the URDF joint limits.");
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult setParametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool next_enabled = enabled_;
    bool next_soft_stop = soft_stop_;
    bool next_feedforward_torque_enabled = feedforward_torque_enabled_;
    bool clear_faults = false;
    std::vector<double> next_desired_positions = desired_positions_;
    std::vector<double> next_desired_velocities = desired_velocities_;
    std::vector<double> next_desired_accelerations = desired_accelerations_;
    std::vector<double> next_model_position_offsets = model_position_offsets_;
    std::vector<double> next_joint_feedforward_enabled = joint_feedforward_enabled_;
    std::vector<double> next_id_kp = id_kp_;
    std::vector<double> next_id_kd = id_kd_;
    std::vector<double> next_max_torque = max_torque_;
    std::vector<double> next_max_torque_rate = max_torque_rate_;
    std::vector<double> next_max_position_error = max_position_error_;
    std::vector<double> next_max_velocity = max_velocity_;
    double next_startup_ramp_sec = startup_ramp_sec_;
    double next_command_kp = command_kp_;
    double next_command_kd = command_kd_;

    try {
      for (const auto& parameter : parameters) {
        const auto& name = parameter.get_name();
        if (name == "enabled") {
          next_enabled = parameter.as_bool();
        } else if (name == "soft_stop") {
          next_soft_stop = parameter.as_bool();
        } else if (name == "feedforward_torque_enabled") {
          next_feedforward_torque_enabled = parameter.as_bool();
        } else if (name == "clear_faults" && parameter.as_bool()) {
          clear_faults = true;
        } else if (name == "desired_positions") {
          const auto requested_positions = expandVector(parameter.as_double_array(), 0.0, name);
          requireFiniteVector(requested_positions, name);
          next_desired_positions = requested_positions;
        } else if (name == "desired_velocities") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireFiniteVector(requested, name);
          next_desired_velocities = requested;
        } else if (name == "desired_accelerations") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireFiniteVector(requested, name);
          next_desired_accelerations = requested;
        } else if (name == "model_position_offsets") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireFiniteVector(requested, name);
          next_model_position_offsets = requested;
        } else if (name == "joint_feedforward_enabled") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireUnitIntervalVector(requested, name);
          next_joint_feedforward_enabled = requested;
        } else if (name == "id_kp") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireNonnegativeVector(requested, name);
          next_id_kp = requested;
        } else if (name == "id_kd") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireNonnegativeVector(requested, name);
          next_id_kd = requested;
        } else if (name == "max_torque") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireNonnegativeVector(requested, name);
          next_max_torque = requested;
        } else if (name == "max_torque_rate") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireNonnegativeVector(requested, name);
          next_max_torque_rate = requested;
        } else if (name == "max_position_error") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireNonnegativeVector(requested, name);
          next_max_position_error = requested;
        } else if (name == "max_velocity") {
          const auto requested = expandVector(parameter.as_double_array(), 0.0, name);
          requireNonnegativeVector(requested, name);
          next_max_velocity = requested;
        } else if (name == "startup_ramp_sec") {
          next_startup_ramp_sec = requireNonnegativeDouble(parameter.as_double(), name);
        } else if (name == "command_kp") {
          next_command_kp = requireNonnegativeDouble(parameter.as_double(), name);
        } else if (name == "command_kd") {
          next_command_kd = requireNonnegativeDouble(parameter.as_double(), name);
        }
      }
      validateDesiredPositions(next_desired_positions, next_model_position_offsets);
      if (enabled_ && next_enabled && next_model_position_offsets != model_position_offsets_) {
        throw std::runtime_error("model_position_offsets can only be changed while the controller is disabled.");
      }
    } catch (const std::exception& error) {
      result.successful = false;
      result.reason = error.what();
      return result;
    }

    if (next_enabled && !enabled_) {
      enabled_since_ = now();
      std::fill(last_command_torque_.begin(), last_command_torque_.end(), 0.0);
    }
    if (!next_enabled || clear_faults) {
      fault_latched_ = false;
      fault_reason_.clear();
    }
    if (clear_faults) {
      std::fill(last_command_torque_.begin(), last_command_torque_.end(), 0.0);
    }

    enabled_ = next_enabled;
    soft_stop_ = next_soft_stop;
    feedforward_torque_enabled_ = next_feedforward_torque_enabled;
    desired_positions_ = next_desired_positions;
    desired_velocities_ = next_desired_velocities;
    desired_accelerations_ = next_desired_accelerations;
    model_position_offsets_ = next_model_position_offsets;
    joint_feedforward_enabled_ = next_joint_feedforward_enabled;
    id_kp_ = next_id_kp;
    id_kd_ = next_id_kd;
    max_torque_ = next_max_torque;
    max_torque_rate_ = next_max_torque_rate;
    max_position_error_ = next_max_position_error;
    max_velocity_ = next_max_velocity;
    startup_ramp_sec_ = next_startup_ramp_sec;
    command_kp_ = next_command_kp;
    command_kd_ = next_command_kd;

    return result;
  }

  void stateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->name.empty()) {
      have_valid_state_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Incoming JointState has no names. Refusing to torque-control by index.");
      return;
    }

    std::unordered_map<std::string, std::size_t> name_to_index;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      name_to_index[msg->name[i]] = i;
    }

    Eigen::VectorXd q = pinocchio::neutral(model_);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    std::vector<double> measured_hardware_positions(active_joints_.size(), 0.0);

    for (std::size_t i = 0; i < active_joints_.size(); ++i) {
      const auto& active_joint = active_joints_[i];
      const auto found = name_to_index.find(active_joint.name);
      if (found == name_to_index.end() || found->second >= msg->position.size()) {
        have_valid_state_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "JointState is missing active joint '%s'. Publishing zero torque.",
                             active_joint.name.c_str());
        return;
      }

      const auto source_index = found->second;
      const double measured_position = msg->position[source_index];

      if (source_index >= msg->velocity.size()) {
        have_valid_state_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "JointState velocity is missing for active joint '%s'. Publishing zero torque.",
                             active_joint.name.c_str());
        return;
      }

      const double measured_velocity = msg->velocity[source_index];
      if (!std::isfinite(measured_position) || !std::isfinite(measured_velocity)) {
        have_valid_state_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "JointState has non-finite position or velocity for '%s'. Publishing zero torque.",
                             active_joint.name.c_str());
        return;
      }

      measured_hardware_positions[i] = measured_position;
      writeModelPosition(q, active_joint, modelPositionFromHardware(i, measured_position));
      v[active_joint.v_index] = measured_velocity;
    }

    for (std::size_t output_index = 0; output_index < output_joint_names_.size(); ++output_index) {
      const auto found = name_to_index.find(output_joint_names_[output_index]);
      if (found != name_to_index.end() && found->second < msg->position.size()) {
        latest_output_positions_[output_index] = msg->position[found->second];
      }
    }

    q_measured_ = q;
    v_measured_ = v;
    measured_hardware_positions_ = measured_hardware_positions;
    last_state_time_ = now();
    have_valid_state_ = true;
  }

  void controlTimerCallback() {
    const auto current_time = now();

    if (!enabled_ || soft_stop_ || fault_latched_ || !haveFreshState(current_time)) {
      publishSafeCommand();
      return;
    }

    if (!stateWithinSafetyLimits()) {
      publishSafeCommand();
      return;
    }

    Eigen::VectorXd commanded_acceleration = Eigen::VectorXd::Zero(model_.nv);
    for (std::size_t i = 0; i < active_joints_.size(); ++i) {
      const auto& joint = active_joints_[i];
      const double position_error = desired_positions_[i] - measured_hardware_positions_[i];
      const double velocity_error = desired_velocities_[i] - v_measured_[joint.v_index];
      commanded_acceleration[joint.v_index] =
          desired_accelerations_[i] + id_kp_[i] * position_error + id_kd_[i] * velocity_error;
    }

    const Eigen::VectorXd tau = pinocchio::rnea(model_, *data_, q_measured_, v_measured_, commanded_acceleration);
    publishTorqueCommand(tau, current_time);
  }

  bool haveFreshState(const rclcpp::Time& current_time) {
    if (!have_valid_state_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No valid JointState yet. Publishing zero torque.");
      return false;
    }

    const double age = (current_time - last_state_time_).seconds();
    if (age > max_state_age_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "JointState timeout: %.3f s old. Publishing zero torque.", age);
      return false;
    }

    return true;
  }

  bool stateWithinSafetyLimits() {
    for (std::size_t i = 0; i < active_joints_.size(); ++i) {
      const auto& joint = active_joints_[i];
      const double position_error = std::abs(desired_positions_[i] - measured_hardware_positions_[i]);
      const double velocity = std::abs(v_measured_[joint.v_index]);
      const double effective_velocity_limit =
          joint.velocity_limit > 0.0 ? std::min(max_velocity_[i], joint.velocity_limit) : max_velocity_[i];

      if (!std::isfinite(position_error) || !std::isfinite(velocity)) {
        latchFault("non-finite measured state");
        return false;
      }
      if (position_error > max_position_error_[i]) {
        latchFault("position error on " + joint.name + " exceeds max_position_error");
        return false;
      }
      if (velocity > effective_velocity_limit) {
        latchFault("velocity on " + joint.name + " exceeds max_velocity");
        return false;
      }
    }

    return true;
  }

  void latchFault(const std::string& reason) {
    if (!fault_latched_) {
      fault_reason_ = reason;
      RCLCPP_ERROR(get_logger(), "Safety fault latched: %s. Publishing zero torque until cleared.", reason.c_str());
    }
    fault_latched_ = true;
  }

  void fillCommandNames(stm2ros::msg::JointControlState& msg) const {
    for (std::size_t i = 0; i < kJointCmdSize; ++i) {
      msg.joint_names[i] = output_joint_names_[i];
    }
  }

  void publishSafeCommand() {
    stm2ros::msg::JointControlState msg;
    fillCommandNames(msg);
    for (std::size_t i = 0; i < kJointCmdSize; ++i) {
      msg.joint_position[i] = latest_output_positions_[i];
      msg.joint_torque[i] = 0.0;
      last_command_torque_[i] = 0.0;
    }
    msg.kp = 0.0;
    msg.kd = 0.0;
    command_pub_->publish(msg);
    last_publish_time_ = now();
  }

  void publishTorqueCommand(const Eigen::VectorXd& tau, const rclcpp::Time& current_time) {
    stm2ros::msg::JointControlState msg;
    fillCommandNames(msg);

    for (std::size_t i = 0; i < kJointCmdSize; ++i) {
      msg.joint_position[i] = latest_output_positions_[i];
      msg.joint_torque[i] = 0.0;
    }

    double dt = 1.0 / control_rate_hz_;
    if (last_publish_time_.nanoseconds() > 0) {
      dt = std::max(1e-4, (current_time - last_publish_time_).seconds());
    }

    double ramp = 1.0;
    if (startup_ramp_sec_ > 0.0) {
      ramp = clamp((current_time - enabled_since_).seconds() / startup_ramp_sec_, 0.0, 1.0);
    }

    for (std::size_t i = 0; i < active_joints_.size(); ++i) {
      const auto& joint = active_joints_[i];
      const auto output_index = joint.output_index;

      double target_torque = tau[joint.v_index] * ramp;
      if (!std::isfinite(target_torque)) {
        latchFault("Pinocchio returned non-finite torque");
        publishSafeCommand();
        return;
      }

      double effective_torque_limit = std::abs(max_torque_[i]);
      if (joint.effort_limit > 0.0 && std::isfinite(joint.effort_limit)) {
        effective_torque_limit = std::min(effective_torque_limit, joint.effort_limit);
      }
      target_torque = clamp(target_torque, -effective_torque_limit, effective_torque_limit);

      const double max_delta = std::abs(max_torque_rate_[i]) * dt;
      const double previous_torque = last_command_torque_[output_index];
      const double limited_torque = clamp(target_torque, previous_torque - max_delta, previous_torque + max_delta);
      const bool joint_torque_enabled = joint_feedforward_enabled_[i] >= 0.5;

      msg.joint_position[output_index] = desired_positions_[i];
      msg.joint_torque[output_index] = (feedforward_torque_enabled_ && joint_torque_enabled) ? limited_torque : 0.0;
    }

    msg.kp = command_kp_;
    msg.kd = command_kd_;

    for (std::size_t i = 0; i < kJointCmdSize; ++i) {
      last_command_torque_[i] = msg.joint_torque[i];
    }
    last_publish_time_ = current_time;
    command_pub_->publish(msg);
  }

  std::string urdf_path_;
  std::string state_topic_;
  std::string command_topic_;
  double control_rate_hz_ = 100.0;
  double max_state_age_sec_ = 0.15;
  bool enabled_ = false;
  bool soft_stop_ = false;
  bool feedforward_torque_enabled_ = false;
  bool fault_latched_ = false;
  std::string fault_reason_;

  std::vector<std::string> active_joint_names_;
  std::vector<std::string> model_joint_names_;
  std::vector<std::string> output_joint_names_;
  std::vector<ActiveJoint> active_joints_;

  std::vector<double> desired_positions_;
  std::vector<double> desired_velocities_;
  std::vector<double> desired_accelerations_;
  std::vector<double> model_position_offsets_;
  std::vector<double> joint_feedforward_enabled_;
  std::vector<double> id_kp_;
  std::vector<double> id_kd_;
  std::vector<double> max_torque_;
  std::vector<double> max_torque_rate_;
  std::vector<double> max_position_error_;
  std::vector<double> max_velocity_;
  double startup_ramp_sec_ = 2.0;
  double command_kp_ = 0.0;
  double command_kd_ = 0.0;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::VectorXd q_measured_;
  Eigen::VectorXd v_measured_;
  std::vector<double> measured_hardware_positions_;

  bool have_valid_state_ = false;
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time enabled_since_{0, 0, RCL_ROS_TIME};
  std::vector<double> latest_output_positions_;
  std::vector<double> last_command_torque_;

  rclcpp::Publisher<stm2ros::msg::JointControlState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<OneLegInverseDynamicsNode>());
  } catch (const std::exception& error) {
    std::cerr << "one_leg_inverse_dynamics_node failed: " << error.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
