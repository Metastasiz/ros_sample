cmake_minimum_required(VERSION 3.8)
project(my_ros_02)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_ros_interface REQUIRED)

add_library(turtlesim_control_component SHARED src/turtle_controller.cpp)
ament_target_dependencies(turtlesim_control_component rclcpp rclcpp_action rclcpp_lifecycle
  rclcpp_components turtlesim geometry_msgs my_ros_interface)
rclcpp_components_register_nodes(turtlesim_control_component "turtle_controller::TurtleController")

add_library(turtlebot_control_component SHARED src/turtle_controller_bot.cpp)
ament_target_dependencies(turtlebot_control_component rclcpp rclcpp_action rclcpp_lifecycle
  rclcpp_components geometry_msgs my_ros_interface)
rclcpp_components_register_nodes(turtlebot_control_component "turtle_controller::TurtleBotController")


install(TARGETS
  turtlesim_control_component
  turtlebot_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
