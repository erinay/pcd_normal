This is a ROS2 package called 'pcd'. It  has two executables, pcd_write and pcd_process. 

pcd_write will about a test_pcd.pcd file. 

pcd_process will subscribe to livox/lidar messages, turn them into point cloud data objects, and estimate the normals of each point, using neighbors close by.

norm_viz visualizes the calculated normal arrows that are not in the vertical direction

    Run with parameter command (--ros-args -p viz_3d:=False) to run normal arrows flattened to 2d
    
voxel_viz visualizes voxel grid as cube

# Notes on Livox Install (Ubuntu 24.04)
### [until LIVOX releases compatible versions]
Install Livox-SDK2 here: https://github.com/tu-darmstadt-ros-pkg/Livox-SDK2/tree/jazzy
Install Livox-ROS2 package here: https://github.com/tu-darmstadt-ros-pkg/livox_ros_driver2?tab=readme-ov-file

# Notes on PCL Package
PCL package currently takes in the point cloud and output normals: https://pointclouds.org/