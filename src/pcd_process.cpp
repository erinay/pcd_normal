#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <cmath>

#include "pcd_process.hpp"

// TODO: Subscribe to lidar data (pointCloud2 topic)
class PointCloudProcesser : public rclcpp::Node
{
    public:
        PointCloudProcesser()
        : Node("point_cloud_processor"){
            // Set up pointcloud subscriber
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "livox/points", 10, std::bind(&PointCloudProcesser::cloud_callback, this, std::placeholders::_1));
            // Set up normal publisher      
            filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
            // Grid map publisher
            grid_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("occupancy_mapping", 10);
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

            // Create GridMap
            const float bound = 6.0; //size of cube to discretize
            const float resoultion = 0.05; //resolution
            grid_map::GridMap gridMap({"normal_x", "normal_y", "occupancy", "elevation"}); // Initializing grid map with a single layer to encode elevation data at eahc poiint
            gridMap.setGeometry(grid_map::Length(bound, bound), resoultion); 
            gridMap.setPosition(grid_map::Position(0.0, 0.0)); // set center at (0, 0)
            gridMap.setFrameId("livox_frame");
            gridMap.setTimestamp(msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec);
            gridMap.add("normal_x", 0.0);
            gridMap.add("normal_y", 0.0);
            gridMap.add("normal_z", 0.0);
            gridMap.add("elevation", 0.0); // Where the arrows live
            gridMap.add("occupancy", 0.0);

            grid_map::Size size = gridMap.getSize();
            int rows = size(0);
            int cols = size(1);

            // Iterate through grid index at bounds to set occupied and normal arrow
            for (int row = 0; row<rows; ++row){
                //Assign occupancy and row for left wall
                grid_map::Index index_l (row, 0);
                gridMap.at("occupancy", index_l) = 1.0;
                gridMap.at("normal_x", index_l) = 0.0;
                gridMap.at("normal_y", index_l) = -1.0;
                //Assign occupancy and row for right wall
                grid_map::Index index_r (row, cols-1);
                gridMap.at("occupancy", index_r) = 1.0;
                gridMap.at("normal_x", index_r) = 0.0;
                gridMap.at("normal_y", index_r) = 1.0;
            } 
            for (int col = 0; col<cols; ++col){
                //Assign occupancy and row for top wall
                grid_map::Index index_t (0,col);
                gridMap.at("occupancy", index_t) = 1.0;
                gridMap.at("normal_x", index_t) = -1.0;
                gridMap.at("normal_y", index_t) = 0.0;
                //Assign occupancy and row for bottom wall
                grid_map::Index index_b (rows-1, col);
                gridMap.at("occupancy", index_b) = 1.0;
                gridMap.at("normal_x", index_b) = 1.0;
                gridMap.at("normal_y", index_b) = 0.0;
            } 

            

            // Creating Hashes for normals
            // std::unordered_map<Key, T, Hash, KeyEqual>
            std::unordered_map<grid_map::Index, int, std::hash<grid_map::Index>, equal_to_grid_map_index> counts;

            // Loop through each point and normal to generate pt and arrow in z=0 plane
            for (size_t i = 0; i < cloud_normals->points.size(); ++i)
            {
                const auto& pt = cloud->points[i];
                const auto& normal = cloud_normals->points[i];

                // Normal Vector Processing
                // // Skip if NaN in point or normal
                if (!pcl::isFinite(pt) || !pcl::isFinite(normal) ||pt.z>1.2) {
                    continue;
                }        
                // // Normalize the normal vector
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

                // Publish ros pointcloud2
                pcl::PointNormal p;
                p.x=pt.x; p.y=pt.y; p.z=0.0;
                p.normal_x=normal.normal_x;
                p.normal_y=normal.normal_y;
                p.normal_z=0.0;
                filtered->points.push_back(p);

                //Renormalize
                float length=std::sqrt(normal.normal_x*normal.normal_x+normal.normal_y*normal.normal_y);
                float renorm_x = normal.normal_x/length;
                float renorm_y = normal.normal_y/length;

                // set gridmap position
                grid_map::Position pos(pt.x, pt.y);
                if (!gridMap.isInside(pos)) continue; //Check if allowable position in gridded world
                grid_map::Index index; // get index for the grid
                gridMap.getIndex(pos, index);
                // moving average
                float norm_x = gridMap.at("normal_x", index);
                float norm_y = gridMap.at("normal_y", index);
                float ct = counts[index];
                gridMap.at("normal_x", index) = ((norm_x*ct)+renorm_x)/(ct+1);
                gridMap.at("normal_y", index) = ((norm_y*ct)+renorm_y)/(ct+1);
                counts[index]+=1;
                gridMap.at("occupancy", index) = 1.0;
            }

            filtered->width = filtered->points.size();
            filtered->height = 1;

            // Convert filtered to sensormsg and publish (for viz)
            sensor_msgs::msg::PointCloud2 filtered_out;
            pcl::toROSMsg(*filtered, filtered_out);
            filtered_out.header = msg->header; 
            
            // Publish
            filtered_pub_ -> publish(filtered_out);
            std::unique_ptr<grid_map_msgs::msg::GridMap> out_msg;
            out_msg = grid_map::GridMapRosConverter::toMessage(gridMap);
            grid_pub_ -> publish(std::move(out_msg));
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcesser>());
    rclcpp::shutdown();
    return 0;
}
