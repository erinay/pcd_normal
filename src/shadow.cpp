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
        // shadowed_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("shadow_map",10);
        openSocket("10.8.44.142", 5005);
    }

private:
    int g1_socket;
    struct sockaddr_in dest_addr;
    void shadow_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        std::vector<int8_t> in_map = msg->data;

        // std::vector<uint8_t> buffer(10); // Create buffer to hold packed data (4 floats = 16 bytes)
        // memcpy(&buffer[0], &(msg->data[0]), 3600);

        sendto(g1_socket, &(msg->data[0]), IMAX*JMAX, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr)); // Send the packed data       
        std::cout<<"hello"<<std::endl;

        // int IMAX = msg->info.height;
        // int JMAX = msg->info.width;
        // fill_to_back(in_map, IMAX, JMAX)
        
        // //publishing
        // auto processed_msg = nav_msgs::msg::OccupancyGrid(*msg);  // Copy metadata
        // processed_msg.data = in_map;

        // shadowed_pub_ -> publish(processed_msg)

    }

    void openSocket(const std::string& ip_address, int port){            
        g1_socket = socket(AF_INET, SOCK_DGRAM, 0); // Create socket
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port);
        inet_pton(AF_INET, ip_address.c_str(), &dest_addr.sin_addr);        
    }        


    // void fill_to_back(float *bound, int *IMAX, int *JMAX){          
    //     float b0[IMAX*JMAX];
    //     memcpy(b0, bound, IMAX*JMAX*sizeof(float));
    //     for(int i = 1; i < IMAX-1; i++){
    //         const float yo = i_to_y(i,y) - y;
    //         const float yo2 = yo * yo;
    //         for(int j = 1; j < JMAX-1; j++){
    //             const float xo = j_to_x(j,x) - x;
    //             const float xo2 = xo * xo;
    //             const float ro2 = xo2 + yo2;
    //             if(!b0[i*JMAX+j]){
    //                 const float tho = atan2f(yo, xo);
    //                 int p_start = 0;
    //                 int p_stop = IMAX;
    //                 if(yo > 0.0f) p_stop = y_to_i(y,y) + 1;
    //                 else p_start = y_to_i(y,y);
    //                 int q_start = 0;
    //                 int q_stop = JMAX;
    //                 if(xo > 0.0f) q_start = x_to_j(x,x);
    //                 else q_stop = x_to_j(x,x) + 1;
    //                 for(int p = p_start; p < p_stop; p++){
    //                     const float yb = i_to_y(p,y) - y;
    //                     const float yb2 = yb * yb;
    //                     for(int q = q_start; q < q_stop; q++){
    //                         if(bound[p*JMAX+q]>0.0f){
    //                             const float xb = j_to_x(q,x) - x;
    //                             const float xb2 = xb * xb;
    //                             const float rb2 = xb2 + yb2;
    //                             const float thb = atan2f(yb, xb);
    //                             const float dr2 = rb2 - ro2;
    //                             const float dth = ang_diff(thb, tho);
    //                             if((dr2>0.0f) && (dth<0.04f) && (dth>-0.04f)) bound[p*JMAX+q] = -1.0f;
    //                         }
    //                     }
    //                 }   
    //             }
    //         }
    //     }
    // }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr shadowed_pub_;

};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Shadow>());
    rclcpp::shutdown();
    return 0;
}
