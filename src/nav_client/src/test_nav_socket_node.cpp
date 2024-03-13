//
// Created by shuxy on 24-3-9.
//

#include "nav_client/test_nav_socket_node.hpp"

namespace nav_client {
    TestNavSocket::TestNavSocket(const rclcpp::NodeOptions &options) : Node("test_nav_socket_node") {
        //set socket
        try {
            client_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (client_socket_fd < 0)
                RCLCPP_ERROR(this->get_logger(), "Error open socket");
            else
                RCLCPP_INFO(this->get_logger(), "connect %s", serv_ip_.c_str());
            port_no_ = this->declare_parameter("port", 51718);
            serv_ip_ = this->declare_parameter("ip", "192.168.123.110");

            serv_addr_.sin_family = AF_INET;
//            serv_addr_.sin_addr.s_addr = inet_addr(serv_ip_.c_str());
            serv_addr_.sin_addr.s_addr = INADDR_ANY;
            serv_addr_.sin_port = htons(port_no_);


            int n_connect = connect(client_socket_fd, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_));
            if (n_connect < 0)
                RCLCPP_ERROR(this->get_logger(), "Error connect server");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Socket Error: %s", ex.what());
            throw ex;
        }

//        receive_thread_ = std::thread(&TestNavSocket::receiveData, this);
        socket_msg_.send_ok = true;
        socket_serv_ = this->create_service<std_srvs::srv::Trigger>(
                "/socket_ok", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                                             std_srvs::srv::Trigger::Response::SharedPtr response)
                                                             { socket_ok(),
                                                                response->success = true;
                                                                return;
                                                             }
                );

        socket_client_ = this->create_client<std_srvs::srv::Trigger>(
                "/socket_ok"
                );

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", rclcpp::SensorDataQoS(),
                std::bind(&TestNavSocket::socketSendGimbal, this, std::placeholders::_1)
        );


    }

    TestNavSocket::~TestNavSocket() {

        close(client_socket_fd);

    }
    void TestNavSocket::socket_ok() {
        socket_msg_.send_ok = true;
    }

    void TestNavSocket::socketSendGimbal(geometry_msgs::msg::Twist::SharedPtr nav_msg) {
        try {
            SendTwistPacket packet;

            // nav2 部分
            packet.header = 0x6A;
            packet.linear_vx = nav_msg->linear.x;
            packet.linear_vy = nav_msg->linear.y;
            packet.linear_vz = nav_msg->linear.z;
            packet.angluar_vz = nav_msg->angular.z;


            std::vector<uint8_t> send_data = toVectorTwist(packet);


//            RCLCPP_INFO(this->get_logger(), "send_data: %d, packet: %d", sizeof (send_data), sizeof(packet));

            // socket send
            bzero(buffer_, BUFFER_SIZE);

            pack(packet, buffer_);

//            int n_sendTO = sendto(client_socket_fd, )

            int n_write = write(client_socket_fd, buffer_, sizeof(packet));

            if (n_write < 0)
                RCLCPP_ERROR(get_logger(), "ERROR writing");
            else
                RCLCPP_INFO(get_logger(), "Send %ld data to %s", msg_cnt++, serv_ip_.c_str());

        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error: %s", ex.what());
        }

    }

    void TestNavSocket::receiveData(nav_interfaces::msg::SocketMsg::SharedPtr socket_msg) {
//        std::vector<uint8_t> recvDate;
//        recvDate.reserve(sizeof (ReceivePacket));
        ReceivePacket recvData;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        socket_client_->async_send_request(request);
        while (rclcpp::ok() && socket_msg_.send_ok) {
            try {

                bzero(buffer_, BUFFER_SIZE);
                int n_read = read(client_socket_fd, buffer_, BUFFER_SIZE);


                RCLCPP_INFO(this->get_logger(), "receive: %s", buffer_);

                if (n_read < 0)
                    RCLCPP_ERROR(this->get_logger(), "Error reading from server");
                else
                    RCLCPP_INFO(this->get_logger(), "Reading data from server: %s", serv_ip_.c_str());
                unpack(recvData, buffer_);
                socket_msg_.send_ok = false;
//                RCLCPP_INFO(this->get_logger(), "aim_x: %f, aim_y: %f, aim_z: %f", recvData.aim_x, recvData.aim_y, recvData.aim_z);

            } catch (const std::exception &ex) {
                RCLCPP_ERROR(get_logger(), "Error receive data: %s", ex.what());

            }


        }


    }
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav_client::TestNavSocket)