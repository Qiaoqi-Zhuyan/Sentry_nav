//
// Created by shuxy on 24-3-12.
//

#ifndef BUILD_TEST_UDP_SOCKET_NODE_HPP
#define BUILD_TEST_UDP_SOCKET_NODE_HPP

// ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

// std
#include "string"
#include "future"
#include "thread"
#include "vector"
#include "memory"


// linux sys
#include "sys/socket.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "cstdint"
#include "unistd.h"

//nav interface
#include "nav_client/nav_packet.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "nav_interfaces/msg/game_status.hpp"

#include "socket_io.hpp"

namespace nav_client{
#define BUFFER_SIZE 256

    class UDPSender : public rclcpp::Node {
    public:
        explicit UDPSender(const rclcpp::NodeOptions &nodeOptions);
        ~UDPSender();
    private:

        void twistSender(geometry_msgs::msg::Twist::SharedPtr nav_msg);

        void receiveMsg();

        int64_t msg_cnt = 0;
        // socket param
        int64_t client_socket_fd;
        uint16_t serv_port_; // 服务器程序端口、IP
        std::string serv_ip_;
        struct sockaddr_in serv_addr_in_;
        char buffer_[BUFFER_SIZE];


        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;

        // 发布比赛状态
        rclcpp::Publisher<nav_interfaces::msg::GameStatus>::SharedPtr game_statu_pub_;

        std::thread receive_thread_;

    };


}

#endif //BUILD_TEST_UDP_SOCKET_NODE_HPP
