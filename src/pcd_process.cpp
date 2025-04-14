#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TODO: Subscribe to lidar data (pointCloud2 topic)
class PointCloudProcesser : public rclpp::Node
{
public:
    PointCloudProcesser()
        :Node('point_cloud_processor', rclcpp::NodeOptions())
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
        {
            // Settin up Publishers
            // RCLCPP_INFO(this->get_logger(), "Setting up publishers");
            voxel_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cluster", 1);
            crop_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("crop_cluster", 1);
        }
}
