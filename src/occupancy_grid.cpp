#include <rclcpp/gos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/msg/marker_array.hpp>

class OccupancyGrid : public rclcpp::Node[
    public:
        OccupancyGrid(): Node("occupancy_grid"){
            marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
                "cloud_normals", 10, 
            )
            voxel_pub_
            voxel_arr_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_arrays", 10);
        }
    private:
    // TODO: Creaste PointNormal() object -> x,y,z, n_x, n_y, n_z

    
]
