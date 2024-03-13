////
//// Created by shuxy on 24-3-5.
////
//#include "nav_client/nav_client.hpp"
//#include "rclcpp/rclcpp.hpp"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "tf2_ros/transform_listener.h"
//#include "sys/socket.h"
//#include "netinet/in.h"
//#include "arpa/inet.h"
//#include "nav_client/nav_packet.hpp"
//
//namespace nav_client{
//
//    CmdVelClient::CmdVelClient(const rclcpp::NodeOptions &options) : Node("cmd_vel_send", options){
//
//        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//                "/cmd_vel", 10 , std::bind(&CmdVelClient::sendCallback, this, std::placeholders::_1)
//                );
//
//
//    }
//
//    void CmdVelClient::sendCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
//
//        struct CmdVelData cmdVelData;
//
//        cmdVelData.linear_vx = cmd_vel_msg->linear.x;
//        cmdVelData.linear_vy = cmd_vel_msg->linear.y;
//        cmdVelData.linear_vz=  cmd_vel_msg->linear.z;
//
//        cmdVelData.angular_vx = cmd_vel_msg->angular.x;
//        cmdVelData.angular_vy = cmd_vel_msg->angular.y;
//        cmdVelData.angluar_vz = cmd_vel_msg->angular.z;
//
//        cmdVelData.v_yaw = cmd_vel_msg->angular.z;
//        cmdVelData.v_pitch = cmd_vel_msg->angular.y;
//        cmdVelData.v_roll = 0.0f;
//
//
////        debug_cmd_vel_pub_ = this->create_publisher<>(
////                "/cmd_vel_msg", 10
////                );
////        debug_cmd_vel_pub_->publish(cmd_vel_msg);
//
//
////        tf2::Quaternion q;
//
//        int sockfd;
//        port_no_ = this->declare_parameter("port", 51717);
//        serv_ip_ = this->declare_parameter("ip", "192.168.123.110");
//
//        char buffer[256];
//        bzero(buffer, 256);
//        std::memcpy(buffer, &cmdVelData, sizeof(CmdVelData));
//
//        struct sockaddr_in server_addr{};
//        struct hostenet *server;
//        server_addr.sin_family = AF_INET;
//        server_addr.sin_port = htons(port_no_);
//        server_addr.sin_addr.s_addr = inet_addr(serv_ip_.c_str());
//
//        sockfd = socket(AF_INET, SOCK_STREAM, 0);
//        if (sockfd < 0)
//            RCLCPP_ERROR(this->get_logger(), "error open");
//
//        if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
//            RCLCPP_ERROR(this->get_logger(), "error connect");
//
//        if (send(sockfd, buffer, sizeof(buffer), 0) < 0)
//            RCLCPP_ERROR(this->get_logger(),"error send");
//
//        close(sockfd);
//
//
//    }
//
//}
//
//#include "rclcpp_components/register_node_macro.hpp"
//
//// Register the component with class_loader.
//// This acts as a sort of entry point, allowing the component to be discoverable when its library
//// is being loaded into a running process.
//RCLCPP_COMPONENTS_REGISTER_NODE(nav_client::CmdVelClient)
