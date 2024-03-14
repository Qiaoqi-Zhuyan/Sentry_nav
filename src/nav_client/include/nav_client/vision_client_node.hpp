////
//// Created by shuxy on 24-3-14.
////
//
//#ifndef BUILD_VISION_CLIENT_NODE_HPP
//#define BUILD_VISION_CLIENT_NODE_HPP
//
//
//// ros2
//#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/publisher.hpp"
//#include "rclcpp/subscription.hpp"
//#include "geometry_msgs/msg/twist.hpp"
//#include "geometry_msgs/msg/transform_stamped.hpp"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "tf2_ros/transform_listener.h"
//#include "tf2_ros/transform_broadcaster.h"
//#include "visualization_msgs/msg/marker.hpp"
//#include "std_msgs/msg/float64.hpp"
//#include "std_srvs/srv/trigger.hpp"
//
//// std
//#include "string"
//#include "future"
//#include "thread"
//#include "vector"
//#include "memory"
//
//// linux sys
//#include "sys/socket.h"
//#include "netinet/in.h"
//#include "arpa/inet.h"
//#include "netdb.h"
//#include "cstdint"
//#include "unistd.h"
//
////nav interface
//#include "nav_client/nav_packet.hpp"
//#include "nav_auto_aim_interfaces/msg/target.hpp"
//#include "nav_auto_aim_interfaces/msg/debug_recv_data.hpp"
//
//
//
//namespace nav_client{
//#define BUFFER_SIZE 256
//    class VisionClientNode : public rclcpp::Node
//    {
//    public:
//        explicit VisionClientNode(const rclcpp::NodeOptions & options);
//
//        ~VisionClientNode() override;
//
//    private:
//        void getParams();
//
//        void receiveData();
//
//        void sendData(nav_auto_aim_interfaces::msg::Target::SharedPtr msg);
//
//        void reopenPort();
//
//        void setParam(const rclcpp::Parameter & param);
//
//        void resetTracker();
//
//        // socket param
//        int64_t client_socket_fd;
//        uint16_t serv_port_; // 服务器程序端口、IP
//        std::string serv_ip_;
//        struct sockaddr_in serv_addr_;
//        char buffer_[BUFFER_SIZE];
//        int64_t send_data_cunt_ = 0, recv_data_cunt_ = 0;
//
//        // Param client to set detect_colr
//        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
//        bool initial_set_param_ = false;
//        uint8_t previous_receive_color_ = 0;
//        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
//        ResultFuturePtr set_param_future_;
//
//        // Service client to reset tracker
//        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
//
//        // Aimimg point receiving from serial port for visualization
//        visualization_msgs::msg::Marker aiming_point_;
//
//        // Broadcast tf from odom to gimbal_link
//        double timestamp_offset_ = 0;
//        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//
//        rclcpp::Subscription<nav_auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
//
//        // For debug usage
//        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
//        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//
//        std::thread receive_thread_;
//    };
//
//
//}
//
//#endif //BUILD_VISION_CLIENT_NODE_HPP
