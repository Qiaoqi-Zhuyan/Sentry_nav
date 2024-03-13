//
// Created by shuxy on 24-3-9.
//

#ifndef BUILD_TEST_NAV_SOCKET_NODE_HPP
#define BUILD_TEST_NAV_SOCKET_NODE_HPP

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
#include "mutex"
#include "queue"

// linux sys
#include "sys/socket.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "cstdint"
#include "unistd.h"

//nav interface
#include "nav_client/nav_packet.hpp"
#include "nav_auto_aim_interfaces/msg/target.hpp"
#include "nav_interfaces/msg/socket_msg.hpp"

namespace nav_client{
#define BUFFER_SIZE 256


    class TestNavSocket : public rclcpp::Node {
    public :
        explicit TestNavSocket(const rclcpp::NodeOptions & options);
        ~TestNavSocket();

    private:
        void socketSendGimbal(geometry_msgs::msg::Twist::SharedPtr nav_msg);

        void receiveData(nav_interfaces::msg::SocketMsg::SharedPtr socket_msg);

        void socket_ok();

        int64_t msg_cnt = 0;
        // socket param
        int64_t client_socket_fd;
        uint16_t port_no_; // 服务器程序端口、IP
        std::string serv_ip_;
        struct sockaddr_in serv_addr_;
        char buffer_[BUFFER_SIZE];

        std::mutex socket_mutex_;
        nav_interfaces::msg::SocketMsg socket_msg_;


        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr socket_serv_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr  socket_client_;

        std::thread receive_thread_;

    };

}

#endif //BUILD_TEST_NAV_SOCKET_NODE_HPP
