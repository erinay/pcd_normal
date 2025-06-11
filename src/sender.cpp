#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstring>
               
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include <arpa/inet.h>

#define IMAX 120
#define JMAX 120

class Shadow : public rclcpp::Node
{
    public: Shadow()
    : Node("shadow"){
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map_convert",10,std::bind(&Shadow::shadow_callback, this, std::placeholders::_1));
        openSocket("169.254.21.26", 5005);
    }

private:
    int g1_socket;
    struct sockaddr_in dest_addr;
    void shadow_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        std::vector<int8_t> in_map = msg->data;

        sendto(g1_socket, &(msg->data[0]), IMAX*JMAX, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr)); // Send the packed data       
        std::cout<<"hello"<<std::endl;
    }

    void openSocket(const std::string& ip_address, int port){            
        g1_socket = socket(AF_INET, SOCK_DGRAM, 0); // Create socket
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port);
        inet_pton(AF_INET, ip_address.c_str(), &dest_addr.sin_addr);        
    }        

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Shadow>());
    rclcpp::shutdown();
    return 0;
}
