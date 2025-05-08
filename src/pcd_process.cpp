#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

//Debug Tools
// #include <sensor_msgs/point_cloud2_iterator.hpp>

inline bool isFinite(const tf2::Quaternion& q) {
    return std::isfinite(q.x()) &&
           std::isfinite(q.y()) &&
           std::isfinite(q.z()) &&
           std::isfinite(q.w());
}

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
            voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid",10);
            filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
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

            // Create PointNomral() Object -> x,y,z, nx, ny, nz
            pcl::PointCloud<pcl::PointNormal>::Ptr filtered (new pcl::PointCloud<pcl::PointNormal>);
           
            // Loop through each point and normal to generate pt and arrow in z=0 plane
            for (size_t i = 0; i < cloud_normals->points.size(); ++i)
            {
                const auto& pt = cloud->points[i];
                const auto& normal = cloud_normals->points[i];

                // Skip if NaN in point or normal
                if (!pcl::isFinite(pt) || !pcl::isFinite(normal)) {
                    continue;
                }

                // Filter if pt is above 1m of cloud
                if(pt.z>1.2){
                    continue;
                }
        
                // Normalize the normal vector
                tf2::Vector3 normal_vec(normal.normal_x, normal.normal_y, 0.0);
                normal_vec.normalize();

                if(normal_vec.getZ()>0.5 || normal_vec.getZ()<-0.5){
                    // RCLCPP_INFO(this->get_logger(), "p_start: z=%.3f", normal_vec.getZ());
                    continue;
                }                

                // Compute rotation from X-axis to normal direction
                tf2::Vector3 x_axis(1.0, 0.0, 0.0);
                tf2::Quaternion q;
                q.setRotation(x_axis.cross(normal_vec), x_axis.angle(normal_vec));
                q.normalize();

                // Skip if NaN in point or normal
                if (!isFinite(q)) {
                    continue;
                }

                pcl::PointNormal p;
                p.x=pt.x; p.y=pt.y; p.z=pt.z;
                p.normal_x=normal.normal_x;
                p.normal_y=normal.normal_y;
                p.normal_z=normal.normal_z;
                filtered->points.push_back(p);
            }

            filtered->width = filtered->points.size();
            filtered->height = 1;

            // Convert filtered to sensormsg and publish
            sensor_msgs::msg::PointCloud2 filtered_out;
            pcl::toROSMsg(*filtered, filtered_out);
            filtered_out.header = msg->header;

            // Voxel Object
            pcl::PointCloud<pcl::PointNormal>::Ptr arr_voxel (new pcl::PointCloud<pcl::PointNormal>);
            pcl::VoxelGrid<pcl::PointNormal> arr;
            arr.setInputCloud(filtered);
            arr.setLeafSize(0.05f, 0.05f, 0.05f);
            arr.filter(*arr_voxel);
            sensor_msgs::msg::PointCloud2 ros_cloud;
            pcl::toROSMsg(*arr_voxel, ros_cloud);
            ros_cloud.header.frame_id = "livox_frame";
            ros_cloud.header.stamp = msg->header.stamp;
            
            // Publish
            voxel_pub_ -> publish(ros_cloud);
            filtered_pub_ -> publish(filtered_out);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcesser>());
    rclcpp::shutdown();
    return 0;
}
