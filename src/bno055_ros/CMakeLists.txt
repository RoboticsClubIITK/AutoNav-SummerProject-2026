cmake_minimum_required(VERSION 3.8)
project(bno055_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# --- Driver library (shared between test_main and imu_node) ---
add_library(bno055_driver
  src/bno055.cpp
)
target_include_directories(bno055_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Standalone test executable (no ROS dependency) ---
add_executable(test_main src/test_main.cpp)
target_link_libraries(test_main bno055_driver)

# --- ROS 2 node executable ---
add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node bno055_driver)
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  test_main
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
