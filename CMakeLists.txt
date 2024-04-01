cmake_minimum_required(VERSION 3.8)
project(kinova_gen3_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(kinova_gen3_control_interfaces REQUIRED)

#for the server
add_library(action_server SHARED
  src/arm_move_to_point_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "KINOVA_GEN3_CONTROL_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "kinova_gen3_control_interfaces"
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "kinova_gen3_control_cpp::ArmMoveToPointServer" EXECUTABLE arm_move_to_point_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#for the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
