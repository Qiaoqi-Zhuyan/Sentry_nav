//
// Created by shuxy on 24-3-9.
//

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
#include "netdb.h"
#include "cstdint"
#include "unistd.h"

//nav interface
#include "nav_client/nav_packet.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/debug_recv_data.hpp"

namespace nav_client{
#define BUFFER_SIZE 256


    class VisionClient : public rclcpp::Node {
    public :
        explicit VisionClient(const rclcpp::NodeOptions & options);
        ~VisionClient();

    private:
        void getParams();

        void receiveData();

        void sendGimbalData(auto_aim_interfaces::msg::Target::SharedPtr target_msg);

        void setParam(const rclcpp::Parameter & param);

        void resetTracker();

        // socket param
        int64_t client_socket_fd;
        uint16_t serv_port_; // 服务器程序端口、IP
        std::string serv_ip_;
        struct sockaddr_in serv_addr_;
        char buffer_[BUFFER_SIZE];
        int64_t send_data_cunt_ = 0, recv_data_cunt_ = 0;

        // detector param
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        bool initial_set_param_ = false;
        uint8_t previous_detect_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        ResultFuturePtr set_param_future_;

        // service client to reset tracker
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

        // tf 坐标系转化 yaw, pitch -> robot_state_publisher -> odom -> gimbal_link
        double timeStamp_offset_ = 0;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

        // 订阅nav_aim_tracker
        rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
        // 订阅nav2 cmd_vel
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

        // debug message
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugRecvData >::SharedPtr recv_pub_;

        // aiming point 可视化
        visualization_msgs::msg::Marker aiming_point_;
        // 接受信息可视化
        auto_aim_interfaces::msg::DebugRecvData recv_data_;

        std::thread receive_thread_;

    };

}


