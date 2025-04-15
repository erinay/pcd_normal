#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TODO: Subscribe to lidar data (pointCloud2 topic)
class PointCloudProcesser : public rclcpp::Node
{
    public:
        PointCloudProcesser()
        : Node("point_cloud_processor"){
            // Set up pointcloud subscriber
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 10, std::bind(&PointCloudProcesser::cloud_callback, this, std::placeholders::_1));
        }
    
    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            // Create PCL Point CLoud object
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcesser>());
    rclcpp::shutdown();
    return 0;
}
