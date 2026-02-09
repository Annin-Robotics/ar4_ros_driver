// simple_pick_place_mgi.cpp
//
// AR4 simple “pick” demo (C++ MoveGroupInterface, MoveIt-only):
//  - Open gripper  (MoveIt named target on ar_gripper)
//  - Move arm SAFE (MoveIt joint targets on ar_manipulator)
//  - Move arm PICK (MoveIt joint targets on ar_manipulator)
//  - Close gripper (MoveIt named target on ar_gripper)
//  - Return SAFE   (MoveIt joint targets on ar_manipulator)
//
// No rclcpp_action / control_msgs dependency needed.
//
// Run:
//   ros2 run ar4_moveit_cpp_demo simple_pick_place_mgi
//
// Assumes MoveIt is already launched (move_group running).

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

static void plan_and_execute(moveit::planning_interface::MoveGroupInterface& mgi,
                             rclcpp::Logger logger,
                             const std::string& label)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  auto plan_code = mgi.plan(plan);
  if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Planning failed: " + label);
  }

  auto exec_code = mgi.execute(plan);
  if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Execution failed: " + label);
  }

  RCLCPP_INFO(logger, "Done: %s", label.c_str());
}

static void set_joint_target(moveit::planning_interface::MoveGroupInterface& arm,
                             const std::vector<std::string>& joint_names,
                             const std::vector<double>& joint_values,
                             const std::string& label)
{
  if (joint_names.size() != joint_values.size()) {
    throw std::runtime_error("Joint name/value size mismatch for " + label);
  }

  // Name->value map avoids ordering surprises.
  std::map<std::string, double> target;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    target[joint_names[i]] = joint_values[i];
  }

  arm.setJointValueTarget(target);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ar4_moveit_simple_pick");
  auto log = node->get_logger();

  // From your SRDF:
  //   Arm group:     ar_manipulator   (joints include joint_1..joint_6 + ee_joint)
  //   Gripper group: ar_gripper       (states: open, closed)
  moveit::planning_interface::MoveGroupInterface arm(node, "ar_manipulator");
  moveit::planning_interface::MoveGroupInterface gripper(node, "ar_gripper");

  // Demo-friendly settings
  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);

  gripper.setPlanningTime(2.0);
  gripper.setNumPlanningAttempts(5);
  gripper.setMaxVelocityScalingFactor(1.0);
  gripper.setMaxAccelerationScalingFactor(1.0);

  // ---- Targets (radians) ----
  // We explicitly command only joint_1..joint_6 to avoid any ee_joint confusion.
  const std::vector<std::string> arm_joints = {
    "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"
  };

  const std::vector<double> safe = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> pick = {0.0, 1.0647, 0.1047, 0.0, 0.4014, 0.0};

  try {
    // Open gripper (MoveIt named state)
    RCLCPP_INFO(log, "Gripper -> open");
    gripper.setNamedTarget("open");
    plan_and_execute(gripper, log, "gripper open");

    // Arm SAFE
    RCLCPP_INFO(log, "Arm -> SAFE (joint target)");
    set_joint_target(arm, arm_joints, safe, "SAFE");
    plan_and_execute(arm, log, "arm SAFE");

    // Arm PICK
    RCLCPP_INFO(log, "Arm -> PICK (joint target)");
    set_joint_target(arm, arm_joints, pick, "PICK");
    plan_and_execute(arm, log, "arm PICK");

    // Close gripper (MoveIt named state)
    RCLCPP_INFO(log, "Gripper -> closed");
    gripper.setNamedTarget("closed");
    plan_and_execute(gripper, log, "gripper closed");

    // Return SAFE
    RCLCPP_INFO(log, "Arm -> SAFE (return)");
    set_joint_target(arm, arm_joints, safe, "SAFE(return)");
    plan_and_execute(arm, log, "arm SAFE (return)");
    
    // Place back in PICK position
    RCLCPP_INFO(log, "Arm -> PICK (joint target)");
    set_joint_target(arm, arm_joints, pick, "PICK");
    plan_and_execute(arm, log, "arm PICK");

    // re-open at end
    RCLCPP_INFO(log, "Gripper -> open (end)");
    gripper.setNamedTarget("open");
    plan_and_execute(gripper, log, "gripper open (end)");
    
    // Return SAFE
    RCLCPP_INFO(log, "Arm -> SAFE (return)");
    set_joint_target(arm, arm_joints, safe, "SAFE(return)");
    plan_and_execute(arm, log, "arm SAFE (return)");

    RCLCPP_INFO(log, "Demo complete.");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(log, "ERROR: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

