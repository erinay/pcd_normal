#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class NormalVisualizer : public rclcpp::Node{
    public:
        NormalVisualizer(): Node("normal_visualizer"){
            voxel_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "voxel_grid", 10, std::bind(&NormalVisualizer::callback, this, std::placeholders::_1));
            voxel_arr_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cube_array", 10);
            }
    private:
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            pcl::PointCloud<pcl::PointNormal>::Ptr voxel_arr (new pcl::PointCloud<pcl::PointNormal>);
            pcl::fromROSMsg(*msg, *voxel_arr);

            visualization_msgs::msg::MarkerArray cube_array;

            int id = 0;

            for (const auto& pt : voxel_arr->points) {

                visualization_msgs::msg::Marker cube;
                cube.header.frame_id = "livox_frame";
                cube.header.stamp = msg->header.stamp;
                cube.ns = "voxels";
                cube.id = id++;
                cube.type = visualization_msgs::msg::Marker::CUBE;
                cube.action = visualization_msgs::msg::Marker::ADD;
                cube.pose.orientation.w = 1.0;
                cube.scale.x = 0.05;
                cube.scale.y = 0.05;
                cube.scale.z = 0.05;
                cube.color.a = 1.0;
                cube.color.r = 0;
                cube.color.g = 1;
                cube.color.b = 0;
                cube.pose.position.x = pt.x;
                cube.pose.position.y = pt.y;
                cube.pose.position.z = pt.z;
                cube_array.markers.push_back(cube);
            }
            voxel_arr_pub_ ->publish(cube_array); 
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_arr_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NormalVisualizer>());
    rclcpp::shutdown();
    return 0;
}
