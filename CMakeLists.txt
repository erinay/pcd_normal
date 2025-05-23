cmake_minimum_required(VERSION 3.8)
project(pcd)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_core REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})  # Add PCL includes
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(/opt/ros/jazzy/include/grid_map_core)

add_executable (pcd_write src/pcd_write.cpp)
ament_target_dependencies(pcd_write PCL rclcpp sensor_msgs)

add_executable (pcd_process src/pcd_process.cpp)
ament_target_dependencies(pcd_process PCL rclcpp sensor_msgs visualization_msgs pcl_conversions tf2 grid_map_pcl grid_map_ros)

add_executable (norm_viz src/normal_visualizer.cpp)
ament_target_dependencies(norm_viz PCL rclcpp sensor_msgs visualization_msgs pcl_conversions)

add_executable (voxel_viz src/voxel_visualizer.cpp)
ament_target_dependencies(voxel_viz PCL rclcpp sensor_msgs visualization_msgs pcl_conversions)

install(TARGETS
  pcd_write
  pcd_process
  norm_viz
  voxel_viz
  DESTINATION lib/${PROJECT_NAME})

ament_package()
