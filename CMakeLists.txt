cmake_minimum_required(VERSION 3.8)
project(pcd)

# finding dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})  # Add PCL includes

add_executable (pcd_write src/pcd_write.cpp)
ament_target_dependencies(pcd_write PCL rclcpp sensor_msgs)

#add_executable (pcd_process src/pcd_process.cpp)
#ament_target_dependencies(pcd_process rclcpp PCL sensor_msgs pcl_conversions)

install(TARGETS
  pcd_write
  #pcd_process
  DESTINATION lib/${PROJECT_NAME})

ament_package()
