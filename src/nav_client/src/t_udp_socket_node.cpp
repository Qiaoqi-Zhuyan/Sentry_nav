//
// Created by shuxy on 24-3-12.
//

#include "nav_client/t_udp_socket_node.hpp"

namespace nav_client {

    UDPSender::UDPSender(const rclcpp::NodeOptions &nodeOptions) : Node("test_udp_sender") {
        
        // socket init
        client_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (client_socket_fd < 0)
            RCLCPP_WARN(this->get_logger(), "Error create socket fd");

        serv_ip_ = this->declare_parameter("serv_ip", "192.168.123.110");
        serv_port_ = this->declare_parameter("serv_port", 51718);

        serv_addr_in_.sin_family = AF_INET;
        serv_addr_in_.sin_addr.s_addr = inet_addr(serv_ip_.c_str());
        serv_addr_in_.sin_port = htons(serv_port_);

        receive_thread_ = std::thread(&UDPSender::receiveMsg, this);

        RCLCPP_INFO(this->get_logger(), "send data to server: %s : %d", serv_ip_.c_str(), serv_port_);

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel_nav", rclcpp::SensorDataQoS(),
                std::bind(&UDPSender::twistSender, this, std::placeholders::_1)
        );
    }


    UDPSender::~UDPSender() {
        close(client_socket_fd);
    }

    void UDPSender::twistSender(geometry_msgs::msg::Twist::SharedPtr nav_msg) {
        try {
            SendTwistPacket packet;

            packet.header = 0x6A;
            packet.linear_vx = nav_msg->linear.x;
            packet.linear_vy = nav_msg->linear.y;
            packet.angular = nav_msg->angular.z;
            packet.v_yaw = nav_msg->angular.z;

            // socket send
            bzero(buffer_, BUFFER_SIZE);

//            RCLCPP_INFO(this->get_logger(), "linaer x: %f, %f %f", packet.linear_vx, packet.linear_vy,
//                        packet.linear_vz);

            socklen_t serv_addr_len = sizeof(serv_addr_in_);

            pack(packet, buffer_);


            int n_sendto = sendto(
                    client_socket_fd,
                    (const char *)buffer_,
                    sizeof(packet),
                    MSG_CONFIRM,
                    (struct sockaddr *) &serv_addr_in_,
                    serv_addr_len);

            perror("");
            if (n_sendto < 0)
                RCLCPP_ERROR(get_logger(), "ERROR sending ");
            else
                RCLCPP_INFO(get_logger(), "Send %ld data to %s", msg_cnt++, serv_ip_.c_str());

        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error: %s", ex.what());
        }
    }

    void UDPSender::receiveMsg() {

        ReceivePacket recvData;
        while (rclcpp::ok()) {
            try {

                bzero(buffer_, BUFFER_SIZE);
                struct sockaddr_in serv_addr_in;
                socklen_t serv_addr_in_len = sizeof(serv_addr_in);
                int n_recvfrom = recvfrom(client_socket_fd,
                                          buffer_,
                                          BUFFER_SIZE,
                                          MSG_WAITALL,
                                          (struct sockaddr *) &serv_addr_in,
                                                  &serv_addr_in_len);

                if (n_recvfrom < 0)
                    RCLCPP_ERROR(this->get_logger(), "Error reading from server");
                else {
                    RCLCPP_INFO(this->get_logger(), "Reading data from server: %s", serv_ip_.c_str());
                    RCLCPP_INFO(this->get_logger(), "receive  buffer size : %d",n_recvfrom);
                }
                unpack(recvData, buffer_);
//                RCLCPP_INFO(this->get_logger(), "aim_x: %f, aim_y: %f, aim_z: %f", recvData.aim_x, recvData.aim_y, recvData.aim_z);

            } catch (const std::exception &ex) {
                RCLCPP_ERROR(get_logger(), "Error receive data: %s", ex.what());

            }


        }


    }

}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav_client::UDPSender)