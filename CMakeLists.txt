cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_write)

# finding dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

add_executable (pcd_write src/pcd_write.cpp)
ament_target_dependencies(pcd_write PCL rclcpp sensor_msgs)

install(TARGETS
  pcd_write
  DESTINATION lib/${PROJECT_NAME})

ament_package()
