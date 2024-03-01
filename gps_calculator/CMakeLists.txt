cmake_minimum_required(VERSION 3.8)
project(gps_calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(l2_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(GpsConverter src/gps_converter.cpp src/node.cpp)
ament_target_dependencies(GpsConverter rclcpp sensor_msgs l2_interfaces std_srvs)

install(TARGETS
  GpsConverter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
