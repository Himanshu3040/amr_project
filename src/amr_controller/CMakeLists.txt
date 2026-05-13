cmake_minimum_required(VERSION 3.5)
project(amr_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(twist_mux_msgs REQUIRED) 
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(amr_interfaces REQUIRED)


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(noisy_controller src/noisy_controller.cpp)
ament_target_dependencies(noisy_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs)

add_executable(lidar_safety_stop src/lidar_safety_stop.cpp)
ament_target_dependencies(lidar_safety_stop rclcpp std_msgs sensor_msgs geometry_msgs twist_mux_msgs rclcpp_action visualization_msgs)

add_executable(ultrasonic_safety_stop src/ultrasonic_safety_stop.cpp)
ament_target_dependencies(ultrasonic_safety_stop rclcpp std_msgs amr_interfaces)

add_executable(safety_arbitrator src/safety_arbitrator.cpp)
ament_target_dependencies(safety_arbitrator rclcpp std_msgs)

add_executable(turbo_scaler src/turbo_scaler.cpp)
ament_target_dependencies(turbo_scaler rclcpp geometry_msgs twist_mux_msgs rclcpp_action)

add_executable(bumper_safety_stop src/bumper_safety_stop.cpp)
ament_target_dependencies(bumper_safety_stop rclcpp sensor_msgs std_msgs)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_controller
  noisy_controller
  twist_relay
  lidar_safety_stop
  ultrasonic_safety_stop
  bumper_safety_stop
  safety_arbitrator
  turbo_scaler
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simple_controller.py
  ${PROJECT_NAME}/noisy_controller.py
  ${PROJECT_NAME}/twist_relay.py
  ${PROJECT_NAME}/lidar_safety_stop.py
  ${PROJECT_NAME}/turbo_scaler.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()