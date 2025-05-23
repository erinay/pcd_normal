#include <rclcpp/gos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/msg/PointCloud2.h>
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_core/GridMap.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

class OccupancyGrid : public rclcpp::Node[
    public:
        OccupancyGrid(): Node("occupancy_grid"){
            marker_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "filtered_cloud", 10, std::bind(&PointCloudProccessor::grid_callback, this,  std::placeholders::_1));
        }
    private:
        void grid_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            grid_map::GridMap gridMap({"grid"}); // Initializing grid map with a single layer
            grid_map::GridMapPclConverter::initializeFromPointcloud(msg, gridMap);
            grid_map::GridMapPclConverter::addLayerFromPointCloud(msg, "grid"); 
        }
]
