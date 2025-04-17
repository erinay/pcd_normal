#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

//Debug Tools
// #include <sensor_msgs/point_cloud2_iterator.hpp>

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
            ne_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cloud_normals", 10);
            // pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_repub",10);
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

            // // Create a Marker message to visualize normals as arrows
            visualization_msgs::msg::MarkerArray marker_array;
            
            // Loop through each point and normal to create an arrow for each
            for (size_t i = 0; i < cloud_normals->points.size(); ++i)
            {
                const auto& pt = cloud->points[i];
                const auto& normal = cloud_normals->points[i];

                // Skip if NaN in point or normal
                if (!pcl::isFinite(pt) || !pcl::isFinite(normal)) {
                    continue;
                }

                // Create a Marker message to visualize normals as arrows
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "livox_frame";
                marker.header.stamp = msg->header.stamp;
                marker.ns = "normals";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;  // Arrow shaft radius
                marker.scale.y = 0.01;   // Arrow head size
                marker.scale.z = 0.01;   // Arrow head size
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;   // Green color for normals
                marker.color.b = 0.0;

                // Set marker position (arrow origin)
                marker.pose.position.x = pt.x;
                marker.pose.position.y = pt.y;
                marker.pose.position.z = pt.z;
                 // Normalize the normal vector
                tf2::Vector3 normal_vec(normal.normal_x, normal.normal_y, normal.normal_z);
                normal_vec.normalize();
                

                if(normal_vec.getZ()>0.6 || normal_vec.getZ()<-0.6){
                    marker.color.r=1.0;
                    marker.color.g=0.0;
                    RCLCPP_INFO(this->get_logger(), "p_start: z=%.3f", normal_vec.getZ());
                }

                // Compute rotation from X-axis to normal direction
                tf2::Vector3 x_axis(1.0, 0.0, 0.0);
                tf2::Quaternion q;
                q.setRotation(x_axis.cross(normal_vec), x_axis.angle(normal_vec));
                q.normalize();

                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();
                
                marker_array.markers.push_back(marker);

                // Log p_start
                // RCLCPP_INFO(this->get_logger(), "p_start: x=%.3f, y=%.3f, z=%.3f", 
                // pt.x,pt.y, pt.z);
                
            }
          
            // // Convert to ROS message (DEBUG Tools)
            // sensor_msgs::msg::PointCloud2 output_msg;
            // pcl::toROSMsg(*cloud, output_msg);
            // output_msg.header = msg->header;  // keep frame_id and timestamp

            // Publish
            ne_pub_->publish(marker_array);
            // pc_pub_->publish(output_msg);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ne_pub_;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcesser>());
    rclcpp::shutdown();
    return 0;
}
