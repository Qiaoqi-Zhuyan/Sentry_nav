//
// Created by shuxy on 24-3-5.
//

//
//#ifndef BUILD_NAV_CLIENT_HPP
//#define BUILD_NAV_CLIENT_HPP
//
//// ros2
//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"
//#include "rclcpp/publisher.hpp"
//
//// std
//#include "string"
//#include "future"
//#include "thread"
//#include "vector"
//
//
//namespace nav_client{
//
//
//    class CmdVelClient : public rclcpp::Node{
//
//    public:
//        CmdVelClient(const rclcpp::NodeOptions & options);
//        ~ CmdVelClient() override;
//    private:
//        int port_no_; // 服务器程序端口、IP
//        std::string serv_ip_;
//
//        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//
//        void sendCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
//
//
//        void sendCallback_debug(const geometry_msgs::msg::Twist::SharedPtr  cmd_vel_msg);
//
//        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr debug_cmd_vel_pub_;
//    };
//
//
//}


//#endif //BUILD_NAV_CLIENT_HPP


//复制
//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"
//
//struct CmdVelData {
//    double linear_x;
//    double linear_y;
//    double angular_z;
//};
//
//class CmdVelSubscriber : public rclcpp::Node
//{
//public:
//    CmdVelSubscriber()
//            : Node("cmd_vel_subscriber")
//    {
//        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//                "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
//                    cmd_vel_data_.linear_x = msg->linear.x;
//                    cmd_vel_data_.linear_y = msg->linear.y;
//                    cmd_vel_data_.angular_z = msg->angular.z;
//                });
//    }
//
//private:
//    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
//    CmdVelData cmd_vel_data_;
//};
//
//int main(int argc, char **argv)
//{
//    rclcpp::init(argc, argv);
//    auto node = std::make_shared<CmdVelSubscriber>();
//    rclcpp::spin(node);
//    rclcpp::shutdown();
//    return 0;
//}


//#include <iostream>
//#include <cstring>
//#include <sys/socket.h>
//#include <netinet/in.h>
//
//struct CmdVelData {
//    double linear_x;
//    double linear_y;
//    double angular_z;
//};
//
//int main()
//{
//    // 创建结构体并设置值
//    CmdVelData cmd_vel_data;
//    cmd_vel_data.linear_x = 1.0;
//    cmd_vel_data.linear_y = 2.0;
//    cmd_vel_data.angular_z = 3.0;
//
//    // 创建缓冲区并复制结构体数据
//    char buffer[sizeof(CmdVelData)];
//    std::memcpy(buffer, &cmd_vel_data, sizeof(CmdVelData));
//
//    // 创建套接字
//    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
//
//    // 设置目标地址和端口
//    struct sockaddr_in dest_addr{};
//    dest_addr.sin_family = AF_INET;
//    dest_addr.sin_port = htons(8888);
//    dest_addr.sin_addr.s_addr = inet_addr("目标IP地址");
//
//    // 连接到目标套接字
//    connect(sockfd, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//
//    // 发送缓冲区中的数据
//    send(sockfd, buffer, sizeof(buffer), 0);
//
//    // 关闭套接字
//    close(sockfd);
//
//    return 0;
//}


//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"
//#include "tf2/transform_listener.h"
//#include <iostream>
//
//class CmdVelSubscriber : public rclcpp::Node
//{
//public:
//    CmdVelSubscriber()
//            : Node("cmd_vel_subscriber")
//    {
//        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//
//        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//                "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
//                    // Transform the twist message to the base_link frame
//                    geometry_msgs::msg::TwistStamped twist_stamped;
//                    twist_stamped.header.frame_id = "base_link";
//                    twist_stamped.header.stamp = this->get_clock()->now();
//                    twist_stamped.twist = *msg;
//
//                    geometry_msgs::msg::TwistStamped transformed_twist;
//                    try {
//                        tf_buffer_->transform(twist_stamped, transformed_twist, "base_link", tf2::durationFromSec(1.0));
//                    } catch (tf2::TransformException &ex) {
//                        RCLCPP_WARN(this->get_logger(), "Failed to transform twist message: %s", ex.what());
//                        return;
//                    }
//
//                    // Convert linear and angular velocities to euler angles
//                    double roll, pitch, yaw;
//                    tf2::Quaternion q(
//                            transformed_twist.twist.angular.x,
//                            transformed_twist.twist.angular.y,
//                            transformed_twist.twist.angular.z);
//                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//
//                    // Print the converted euler angles
//                    std::cout << "Roll: " << roll << std::endl;
//                    std::cout << "Pitch: " << pitch << std::endl;
//                    std::cout << "Yaw: " << yaw << std::endl;
//                });
//    }
//
//private:
//    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
//    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//};
//
//int main(int argc, char **argv)
//{
//    rclcpp::init(argc, argv);
//    auto node = std::make_shared<CmdVelSubscriber>();
//    rclcpp::spin(node);
//    rclcpp::shutdown();
//    return 0;
//}


















//#include <netdb.h>
//#include <netinet/in.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <sys/socket.h>
//#include <sys/types.h>
//#include <unistd.h>
//
//void error(const char *msg) {
//    perror(msg);
//    exit(0);
//}
//
//int main(int argc, char *argv[]) {
//    int sockfd, portno, n;
//    struct sockaddr_in serv_addr;
//    struct hostent *server;
//
//    char buffer[256];
//    if (argc < 3) {
//        fprintf(stderr, "usage %s hostname port\n", argv[0]);
//        exit(0);
//    }
//
//    portno = atoi(argv[2]);
//
//    sockfd = socket(AF_INET, SOCK_STREAM, 0);
//    if (sockfd < 0) {
//        error("ERROR opening socket");
//    }
//
//    server = gethostbyname(argv[1]);
//    if (server == NULL) {
//        fprintf(stderr, "ERROR, no such host\n");
//        exit(0);
//    }
//
//    bzero((char *)&serv_addr, sizeof(serv_addr));
//    serv_addr.sin_family = AF_INET;
//    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,
//          server->h_length);
//    serv_addr.sin_port = htons(portno);
//
//    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
//        error("ERROR connecting");
//    }
//
//    while (1) {
//
//        printf("Please enter the message: ");
//        bzero(buffer, 256);
//        fgets(buffer, 255, stdin);
//        n = write(sockfd, buffer, strlen(buffer));
//
//        if (n < 0) {
//            error("ERROR writing to socket");
//        }
//
//        bzero(buffer, 256);
//        n = read(sockfd, buffer, 255);
//
//        if (n < 0) {
//            error("ERROR reading from socket");
//        }
//
//        printf("%s\n", buffer);
//    }
//
//    close(sockfd);
//    return 0;
//}
//
//








