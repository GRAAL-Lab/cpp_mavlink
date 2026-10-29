cmake_minimum_required(VERSION 3.8)
project(cpp_mavlink)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavlink REQUIRED)      # header-only MAVLink C library (ros-<distro>-mavlink)
find_package(OpenCV REQUIRED)
find_package(mavlink_msg REQUIRED) # enable when using the custom messages

# SYSTEM include: generated MAVLink headers are noisy under -Wall -Wextra
include_directories(include)
include_directories(SYSTEM ${mavlink_INCLUDE_DIRS})

add_executable(mavlink_publisher src/mavlink_publisher_node.cpp)
ament_target_dependencies(mavlink_publisher
  rclcpp std_msgs geometry_msgs sensor_msgs mavros_msgs mavlink_msg)


install(TARGETS mavlink_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
