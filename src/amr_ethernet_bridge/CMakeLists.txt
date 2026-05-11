cmake_minimum_required(VERSION 3.8)
project(amr_ethernet_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amr_interfaces REQUIRED)

# ── TCP Bridge Node ───────────────────────────────────────────────────────────
add_executable(tcp_bridge_node src/tcp_bridge_node.cpp)

ament_target_dependencies(tcp_bridge_node
  rclcpp
  std_msgs
)

# Link POSIX socket library (already available on Linux, explicit for clarity)
target_link_libraries(tcp_bridge_node pthread)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS tcp_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
