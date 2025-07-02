#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>  
#include <pcl/common/transforms.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <cmath>
#include <Eigen/Dense>

#include "pcd_process.hpp"

double ANGLE=12.6*M_PI/180.0;

class PointCloudProcesser : public rclcpp::Node
{
    public:
        PointCloudProcesser()
        : Node("point_cloud_processor"){
            // Set up pointcloud subscriber
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "livox/points", 10, std::bind(&PointCloudProcesser::cloud_callback, this, std::placeholders::_1));
            // TEMP reformatted angle publisher
            tf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tf_cloud",10);
            // Set up normal publisher      
            filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
            // Grid map publisher
            grid_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("occupancy_mapping", 10);
        }
    
    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

            // Create PCL Point CLoud object
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*msg, *cloud);

            // Define rotation matrix (e.g., rotate 30° around X)
            // Manually fill in the rotation matrix values from Python
            Eigen::Matrix3f R;
            R << std::cos(ANGLE), 0.0f, std::sin(ANGLE),
            0.0f,  1.0f, 0.0f,
            -std::sin(ANGLE), 0.0f, std::cos(ANGLE);
            // Create affine transformation
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.linear() = R;  // No translation, only rotation
            // Apply transform
            pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cloud, *rotated_cloud, transform);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZI>);
            removeGroundPlane(rotated_cloud, cloud_no_ground);

            // Convert filtered to sensormsg and publish (for viz)
            sensor_msgs::msg::PointCloud2 tf_out;
            pcl::toROSMsg(*cloud_no_ground, tf_out);
            tf_out.header = msg->header; 
            tf_pub_ -> publish(tf_out);

            // Create the normal estimation class, and pass the input dataset to it
            pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
            ne.setInputCloud(cloud_no_ground);
            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
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
                const auto& pt = cloud_no_ground->points[i];
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
                p.x=pt.x; p.y=pt.y; p.z=pt.z;
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
                // if (counts[index]>=5){
                gridMap.at("occupancy", index) = 1.0;
                // }
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

        void removeGroundPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_removed_cloud)  {

            pcl::PointIndices::Ptr ground_candidate_indices(new pcl::PointIndices);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_candidates(new pcl::PointCloud<pcl::PointXYZI>);
            
            for (size_t i = 0; i < input_cloud->points.size(); ++i) {
                const auto& pt = input_cloud->points[i];
                if (!pcl::isFinite(pt)) continue;
                if (pt.z < 0.0) {
                    ground_candidates->points.push_back(pt);
                    ground_candidate_indices->indices.push_back(i);
                }
            }


            ground_candidates->width = ground_candidates->points.size();
            ground_candidates->height = 1;
            ground_candidates->is_dense = true;

            if (ground_candidates->empty()) {
                std::cout << "No ground candidates found under Z threshold." << std::endl;
                *ground_removed_cloud = *input_cloud;  // Return unmodified cloud
                return;
            }
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZI> seg;            
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));        // Prefer planes perpendicular to Z (i.e. horizontal)
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.05);  // Adjust this threshold based on sensor noise
            seg.setInputCloud(ground_candidates);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                // break;
            }

             pcl::PointIndices::Ptr full_cloud_inliers(new pcl::PointIndices);
            for (int idx : inliers->indices) {
                full_cloud_inliers->indices.push_back(ground_candidate_indices->indices[idx]);
            }

            // Extract non-ground (outlier) points
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(input_cloud);
            extract.setIndices(full_cloud_inliers);
            extract.setNegative(true);  // True = remove inliers (i.e., remove the plane)
            extract.filter(*ground_removed_cloud);

            // std::cout << "Plane coefficients: ";
            // for (float c : coefficients->values) std::cout << c << " ";
            std::cout << std::endl;
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tf_pub_;
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
