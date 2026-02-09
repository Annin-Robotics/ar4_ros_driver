cmake_minimum_required(VERSION 3.8)
project(ar4_moveit_cpp_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(simple_pick_place_mgi src/simple_pick_place_mgi.cpp)
ament_target_dependencies(simple_pick_place_mgi
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS simple_pick_place_mgi
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

