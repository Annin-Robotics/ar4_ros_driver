#include <annin_ar4_driver/ar_hardware_interface.hpp>
#include <sstream>

namespace annin_ar4_driver {

hardware_interface::CallbackReturn ARHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  init_variables();

  // init motor driver
  std::string serial_port = info_.hardware_parameters.at("serial_port");
  std::string ar_model = info_.hardware_parameters.at("ar_model");
  std::string velocity_control_p =
      info_.hardware_parameters.at("velocity_control_enabled");
  bool velocity_control_enabled =
      velocity_control_p == "True" || velocity_control_p == "true";
  int baud_rate = 115200;
  bool success = driver_.init(ar_model, serial_port, baud_rate,
                              info_.joints.size(), velocity_control_enabled);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }
     

  

  // calibrate joints if needed
  bool calibrate = info_.hardware_parameters.at("calibrate") == "True";
  if (calibrate) {
    // run calibration
    RCLCPP_INFO(logger_, "Running joint calibration...");
    std::string calib_sequence = info_.hardware_parameters.at("calib_sequence");
    if (calib_sequence.length() != 7) {
      RCLCPP_ERROR(logger_, "Invalid calib_sequence length: %zu. Expected: 7",
                   calib_sequence.length());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!driver_.calibrateJoints(calib_sequence)) {
      RCLCPP_INFO(logger_, "calibration failed.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "calibration succeeded.");
  }
  
  
  // Create a small internal node to host services (shares the same process as        controller_manager)

  service_node_ = std::make_shared<rclcpp::Node>("ar4_hw_services_" + ar_model);


// Service: calibrate joints by mask (e.g. "000011")
calib_mask_srv_ =
    service_node_->create_service<annin_ar4_driver::srv::CalibrateMask>(
        "calibrate_mask",
        [this](
            const std::shared_ptr<annin_ar4_driver::srv::CalibrateMask::Request>
                req,
            std::shared_ptr<annin_ar4_driver::srv::CalibrateMask::Response>
                res) {
          // Enable special mode so ros2_control write() stops streaming MT/MV
          special_mode_.store(true);
          const auto clear_special =
              std::unique_ptr<void, std::function<void(void*)>>(
                  (void*)1, [this](void*) { special_mode_.store(false); });

          // Basic validation
          if (req->mask.size() != 6) {
            res->success = false;
            res->message = "Mask must be exactly 6 chars (J1..J6), e.g. 000011";
            return;
          }
          for (char c : req->mask) {
            if (c != '0' && c != '1') {
              res->success = false;
              res->message = "Mask must contain only '0' or '1'.";
              return;
            }
          }

          // Refuse if estopped
          if (driver_.isEStopped()) {
            res->success = false;
            res->message =
                "Robot is E-stopped. Reset E-stop before calibrating.";
            return;
          }

          // Run calibration by mask (blocks until JC ack OR timeout/ER)
          const bool ok = driver_.calibrateMask(req->mask);
          res->success = ok;
          res->message = ok ? "Calibration (mask) completed/acknowledged."
                            : "Calibration failed (timeout/ER/no ack).";
        });

// Service: park
park_srv_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "park",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      // Enable special mode so ros2_control write() stops streaming MT/MV
      special_mode_.store(true);
      const auto clear_special =
          std::unique_ptr<void, std::function<void(void*)>>(
              (void*)1, [this](void*) { special_mode_.store(false); });

      // Refuse if estopped
      if (driver_.isEStopped()) {
        res->success = false;
        res->message = "Robot is E-stopped. Reset E-stop before parking.";
        return;
      }

      // Run park (blocks until PK ack OR timeout/ER)
      const bool ok = driver_.park();
      res->success = ok;
      res->message =
          ok ? "Park completed/acknowledged." : "Park failed (timeout/ER/no ack).";
    });
  



// Spin services in a dedicated thread (so ros2_control update loop isn't blocked)
  service_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  service_exec_->add_node(service_node_);

  service_thread_ = std::thread([this]() {
    service_exec_->spin();
  });


  return hardware_interface::CallbackReturn::SUCCESS;
}

void ARHardwareInterface::init_variables() {
  // resize vectors
  int num_joints = info_.joints.size();
  actuator_pos_commands_.resize(num_joints);
  actuator_vel_commands_.resize(num_joints);
  actuator_positions_.resize(num_joints);
  actuator_velocities_.resize(num_joints);
  joint_positions_.resize(num_joints);
  joint_velocities_.resize(num_joints);
  joint_efforts_.resize(num_joints);
  joint_position_commands_.resize(num_joints);
  joint_velocity_commands_.resize(num_joints);
  joint_effort_commands_.resize(num_joints);
  joint_offsets_.resize(num_joints);
  for (int i = 0; i < num_joints; ++i) {
    joint_offsets_[i] =
        std::stod(info_.joints[i].parameters["position_offset"]);
  }
}

hardware_interface::CallbackReturn ARHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // Reset Estop (if any)
  bool success = driver_.resetEStop();
  if (!success) {
    RCLCPP_ERROR(logger_,
                 "Cannot activate. Hardware E-stop state cannot be reset.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ARHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position",
                                  &joint_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                  &joint_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ARHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position",
                                    &joint_position_commands_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                    &joint_velocity_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type ARHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  driver_.getJointPositions(actuator_positions_);
  driver_.getJointVelocities(actuator_velocities_);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // apply offsets, convert from deg to rad for moveit
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
    joint_velocities_[i] = degToRad(actuator_velocities_[i]);
  }
  return hardware_interface::return_type::OK;
}

ARHardwareInterface::~ARHardwareInterface() {
  if (service_exec_) {
    service_exec_->cancel();
  }
  if (service_thread_.joinable()) {
    service_thread_.join();
  }
}

hardware_interface::return_type ARHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  // If a special command (park/calibration) is running, do NOT stream MT/MV.
  // Streaming will override PK/JM and prevent motion.
  if (special_mode_.load()) {
    // Optionally still check estop status by polling state, but do NOT send commands.
    // driver_.getJointPositions(actuator_positions_);
    // driver_.getJointVelocities(actuator_velocities_);

    if (driver_.isEStopped()) {
      std::string logWarn =
          "Hardware in EStop state. To reset the EStop "
          "reactivate the hardware component using 'ros2 "
          "run annin_ar4_driver reset_estop.sh <ar_model>'.";
      RCLCPP_WARN(logger_, logWarn.c_str());
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // convert from rad to deg, apply offsets
    actuator_pos_commands_[i] =
        radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
    actuator_vel_commands_[i] = radToDeg(joint_velocity_commands_[i]);
  }

  driver_.update(actuator_pos_commands_, actuator_vel_commands_,
                 actuator_positions_, actuator_velocities_);

  if (driver_.isEStopped()) {
    std::string logWarn =
        "Hardware in EStop state. To reset the EStop "
        "reactivate the hardware component using 'ros2 "
        "run annin_ar4_driver reset_estop.sh <ar_model>'.";
    RCLCPP_WARN(logger_, logWarn.c_str());

    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}


}  // namespace annin_ar4_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(annin_ar4_driver::ARHardwareInterface,
                       hardware_interface::SystemInterface)
