#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class NormalVisualizer : public rclcpp::Node{
    public:
        NormalVisualizer(): Node("normal_visualizer"){
            this->declare_parameter("viz_3d", true);
            this->get_parameter("viz_3d", viz_3d_);
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "filtered_cloud", 10, std::bind(&NormalVisualizer::callback, this, std::placeholders::_1));
            ne_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cloud_normals", 10);
        }
    private:
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            pcl::PointCloud<pcl::PointNormal>::Ptr normal_arr (new pcl::PointCloud<pcl::PointNormal>);
            pcl::fromROSMsg(*msg, *normal_arr);

            visualization_msgs::msg::MarkerArray marker_array;

            int id = 0;
            for (const auto& pt : normal_arr->points) {

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "livox_frame";
                marker.header.stamp = msg->header.stamp;
                marker.ns = "normals";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.01;  // Arrow shaft radius
                marker.scale.y = 0.01;   // Arrow head size
                marker.scale.z = 0.01;   // Arrow head size
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;   // Green color for normals
                marker.color.b = 0.0;

                marker.points.resize(2);
                marker.points[0].x = pt.x;
                marker.points[0].y = pt.y;
                marker.points[1].x = pt.x+0.25*pt.normal_x;
                marker.points[1].y = pt.y+0.25*pt.normal_y;
                if (viz_3d_){
                    marker.points[0].z = pt.z;
                    marker.points[1].z = pt.z+0.25*pt.normal_z;
                }
                else{
                    marker.points[0].z = 0;
                    marker.points[1].z = 0;
                }
                marker_array.markers.push_back(marker);
            }
            ne_pub_ -> publish(marker_array);
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ne_pub_;
        bool viz_3d_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NormalVisualizer>());
    rclcpp::shutdown();
    return 0;
}
