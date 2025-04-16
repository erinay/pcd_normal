#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>

#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TODO: Subscribe to lidar data (pointCloud2 topic)
class PointCloudProcesser : public rclcpp::Node
{
    public:
        PointCloudProcesser()
        : Node("point_cloud_processor"){
            // Set up pointcloud subscriber
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "livox/lidar", 10, std::bind(&PointCloudProcesser::cloud_callback, this, std::placeholders::_1));
            // Set up normal publisher
            ne_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cloud_normals", 10);
        }
    
    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            // Create PCL Point CLoud object
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);

            // Create the normal estimation class, and pass the input dataset to it
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(cloud);

            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
            ne.setSearchMethod(tree);

            // Output datasets
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

            // Use all neighbors in a sphere of radius 3cm
            ne.setRadiusSearch(0.03);

            // Compute the features
            ne.compute(*cloud_normals);

            // Create a Marker message to visualize normals as arrows
            visualization_msgs::msg::Marker markers;
            markers.header.frame_id = msg->header.frame_id;
            markers.header.stamp = msg->header.stamp;
            markers.ns = "normals";
            markers.id = 0;
            markers.type = visualization_msgs::msg::Marker::ARROW;
            markers.action = visualization_msgs::msg::Marker::ADD;
            markers.pose.orientation.w = 1.0;
            markers.scale.x = 0.05;  // Arrow shaft radius
            markers.scale.y = 0.1;   // Arrow head size
            markers.scale.z = 0.1;   // Arrow head size
            markers.color.a = 1.0;
            markers.color.r = 0.0;
            markers.color.g = 1.0;   // Green color for normals
            markers.color.b = 0.0;

            // // Combine XYZ and Normals
            // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            // pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);

            // Loop through each point and normal to create an arrow for each
            for (size_t i = 0; i < cloud_normals->points.size(); ++i)
            {
                const auto& pt = cloud->points[i];
                const auto& normal = cloud_normals->points[i];

                // Skip if NaN in point or normal
                if (!pcl::isFinite(pt) || !pcl::isFinite(normal)) {
                    continue;
                }
                geometry_msgs::msg::Point p_start;
                p_start.x = cloud->points[i].x;
                p_start.y = cloud->points[i].y;
                p_start.z = cloud->points[i].z;

                geometry_msgs::msg::Point p_end;
                p_end.x = p_start.x + cloud_normals->points[i].normal_x * 0.1;  // Scale by 0.1 for visibility
                p_end.y = p_start.y + cloud_normals->points[i].normal_y * 0.1;
                p_end.z = p_start.z + cloud_normals->points[i].normal_z * 0.1;

                markers.points.push_back(p_start);
                markers.points.push_back(p_end);

                 // Log p_start
                RCLCPP_INFO(this->get_logger(), "p_start: x=%.3f, y=%.3f, z=%.3f", 
                p_start.x, p_start.y, p_start.z);
            }

            // // Convert to ROS message
            // sensor_msgs::msg::PointCloud2 output_msg;
            // pcl::toROSMsg(*cloud_with_normals, output_msg);
            // output_msg.header = msg->header;  // keep frame_id and timestamp

            // Publish
            ne_pub_->publish(markers);
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ne_pub_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcesser>());
    rclcpp::shutdown();
    return 0;
}
