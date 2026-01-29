#pragma once

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "annin_ar4_driver/teensy_driver.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "annin_ar4_driver/srv/calibrate_mask.hpp"
#include <atomic>




namespace annin_ar4_driver {
class ARHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ARHardwareInterface);

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  ~ARHardwareInterface() override;

 private:
  rclcpp::Logger logger_ = rclcpp::get_logger("annin_ar4_driver");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);
  
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr service_exec_;
  std::thread service_thread_;

  rclcpp::Service<annin_ar4_driver::srv::CalibrateMask>::SharedPtr calib_mask_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr park_srv_;


  // Motor driver
  TeensyDriver driver_;
  std::atomic_bool special_mode_{false};
  std::vector<double> actuator_pos_commands_;
  std::vector<double> actuator_vel_commands_;
  std::vector<double> actuator_positions_;
  std::vector<double> actuator_velocities_;

  // Shared memory
  std::vector<double> joint_offsets_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;

  // Misc
  void init_variables();
  static constexpr double PI = 3.14159265358979323846;

  double degToRad(double deg) { return deg * PI / 180.0; }
  double radToDeg(double rad) { return rad * 180.0 / PI; }


};
}  // namespace annin_ar4_driver
