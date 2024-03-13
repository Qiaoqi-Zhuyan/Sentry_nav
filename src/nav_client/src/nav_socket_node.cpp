//
// Created by shuxy on 24-3-8.
//

//ros2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/qos.hpp"

// std;
#include "cstdint"
#include "functional"
#include "map"
#include "memory"
#include "string"
#include "vector"

#include "nav_client/nav_socket_node.hpp"

namespace nav_client {
    NavClient::NavClient(const rclcpp::NodeOptions &options) : Node("nav_client_node", options) {


        //set socket
        try {
            client_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (client_socket_fd < 0)
                RCLCPP_ERROR(this->get_logger(), "Error open socket");
            port_no_ = this->declare_parameter("port", 8080);
            serv_ip_ = this->declare_parameter("ip", "127.0.0.1");
            serv_addr_.sin_family = AF_INET;
            serv_addr_.sin_addr.s_addr = inet_addr(serv_ip_.c_str());
            serv_addr_.sin_port = htons(port_no_);
            int n_connect = connect(client_socket_fd, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_));
            if (n_connect < 0)
                RCLCPP_ERROR(this->get_logger(), "Error connect server");
            else
                RCLCPP_INFO(this->get_logger(), "Connect server: %s", serv_ip_.c_str());
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Socket Error: %s", ex.what());
            throw ex;
        }

        receive_thread_ = std::thread(&NavClient::receiveData, this);

        // tf2 广播 对坐标系进行转化
        timeStamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
        tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
        detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "nav_armor_detector");
        // 可视化发送信息
        recv_pub_ = this->create_publisher<nav_auto_aim_interfaces::msg::DebugRecvData>("/receive", 10);

        // tracker reset service client
        reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

        aiming_point_.header.frame_id = "odom";
        aiming_point_.ns = "aiming_point";
        aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
        aiming_point_.action = visualization_msgs::msg::Marker::ADD;
        aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
        aiming_point_.color.r = 1.0;
        aiming_point_.color.g = 1.0;
        aiming_point_.color.b = 1.0;
        aiming_point_.color.a = 1.0;
        aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);


        // 自瞄 tracker 订阅
        target_sub_ = this->create_subscription<nav_auto_aim_interfaces::msg::Target>(
                "/tracker/target", rclcpp::SensorDataQoS(),
                std::bind(&NavClient::sendGimbalData, this, std::placeholders::_1)
        );

        // nav2 cmd_vel 订阅
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", rclcpp::SensorDataQoS(),
                std::bind(&NavClient::sendTwistData, this, std::placeholders::_1)
        );

    }

    NavClient::~NavClient() {

        close(client_socket_fd);
        close(serv_socket_fd);

    }


    void NavClient::sendGimbalData(nav_auto_aim_interfaces::msg::Target::SharedPtr target_msg) {

        const static std::map<std::string, uint8_t> id_uint8_map{
                {"",        0},
                {"outpost", 0},
                {"1",       1},
                {"1",       1},
                {"2",       2},
                {"3",       3},
                {"4",       4},
                {"5",       5}
        };

        try {
            SendGimbalPacket packet;

            // 自瞄部分
            packet.header = 0x5A;
            packet.tracking = target_msg->tracking;
            packet.id = id_uint8_map.at(target_msg->id);
            packet.armors_num = target_msg->armors_num;
            packet.x = target_msg->position.x;
            packet.y = target_msg->position.y;
            packet.z = target_msg->position.z;
            packet.yaw = target_msg->yaw;
            packet.vx = target_msg->velocity.x;
            packet.vy = target_msg->velocity.y;
            packet.vz = target_msg->velocity.z;
            packet.v_yaw = target_msg->v_yaw;
            packet.r1 = target_msg->radius_1;
            packet.r2 = target_msg->radius_2;
            packet.dz = target_msg->dz;

            std::vector<uint8_t> send_data = toVectorGimbal(packet);

            std_msgs::msg::Float64 latency;
            latency.data = (this->now() - target_msg->header.stamp).seconds() * 1000.0f;
            RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
            latency_pub_->publish(latency);


            // socket send
            bzero(buffer_, BUFFER_SIZE);
            for (size_t i = 0; i < sizeof (send_data); i++)
                buffer_[i] = send_data[i];
            int n_send = send(client_socket_fd, buffer_, sizeof (buffer_), 0);
            if (n_send < 0)
                RCLCPP_ERROR(get_logger(), "Error Send");



        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error: %s", ex.what());

        }


    }

    void NavClient::sendTwistData(geometry_msgs::msg::Twist::SharedPtr nav_msg) {

        try {
            SendTwistPacket packet;

            // nav2 部分
            packet.header = 0x6A;
            packet.linear_vx = nav_msg->linear.x;
            packet.linear_vy = nav_msg->linear.y;
            packet.angular = nav_msg->angular.z;
            packet.v_yaw = nav_msg->angular.z;

            std::vector<uint8_t> send_data = toVectorTwist(packet);

            // socket send
            bzero(buffer_, BUFFER_SIZE);
            std::memcpy(buffer_, &send_data, sizeof(send_data));

            int n_write = write(client_socket_fd, buffer_, sizeof (buffer_));
            if (n_write < 0)
                RCLCPP_ERROR(this->get_logger(), "ERROR nav2 data writing");
            else
                RCLCPP_INFO(this->get_logger(), "Send data %ld to %s", ++send_data_cunt_ ,serv_ip_.c_str());

        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error: %s", ex.what());

        }


    }

    void NavClient::receiveData() {
        std::vector<uint8_t> recvData;
        recvData.reserve(sizeof(ReceivePacket));

        while (rclcpp::ok()) {
            try {

                bzero(buffer_, BUFFER_SIZE);
                int n_read = (client_socket_fd, buffer_, BUFFER_SIZE);

                if (n_read < 0)
                    RCLCPP_ERROR(this->get_logger(), "Error reading from server");
                else
                    RCLCPP_INFO(this->get_logger(),"Reading data %ld from server %s", ++recv_data_cunt_, serv_ip_.c_str());

                for(int i = 0; i < sizeof (buffer_); i++)
                    recvData[i] = buffer_[i];




                ReceivePacket recvPacket = fromVector(recvData);
                setParam(rclcpp::Parameter("detect_color", recvPacket.detect_color));
                detect_color_ = recvPacket.detect_color;

                if (recvPacket.reset_tracker)
                    resetTracker();

                geometry_msgs::msg::TransformStamped t;
                timeStamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timeStamp_offset_);
                t.header.frame_id = "odom";
                t.child_frame_id = "gimbal_link";
                tf2::Quaternion q;
                q.setRPY(recvPacket.roll, recvPacket.pitch, recvPacket.yaw);
                t.transform.rotation = tf2::toMsg(q);
                tf2_broadcaster_->sendTransform(t);

                if (abs(recvPacket.aim_x) > 0.01) {
                    aiming_point_.header.stamp = this->now();
                    aiming_point_.pose.position.x = recvPacket.aim_x;
                    aiming_point_.pose.position.y = recvPacket.aim_y;
                    aiming_point_.pose.position.z = recvPacket.aim_z;
                    marker_pub_->publish(aiming_point_);
                }

                // 接受信息可视化
                recv_data_.aim_x = recvPacket.aim_x;
                recv_data_.aim_y = recvPacket.aim_y;
                recv_data_.aim_z = recvPacket.aim_z;
                recv_data_.yaw = recvPacket.yaw;
                recv_data_.pitch = recvPacket.pitch;
                recv_data_.roll = recvPacket.roll;
                recv_data_.reset_tracker = recvPacket.reset_tracker;
                recv_data_.reserved = recvPacket.reserved;
                recv_data_.detect_color = recvPacket.detect_color;

                recv_pub_->publish(recv_data_);




            } catch (const std::exception &ex) {
                RCLCPP_ERROR(get_logger(), "Error receive data: %s", ex.what());
            }


        }
    }

    void NavClient::resetTracker() {
        if (!reset_tracker_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        reset_tracker_client_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Reset tracker");
    }

    void NavClient::setParam(const rclcpp::Parameter &param) {
        if (!detector_param_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
            return;
        }
        if (
                !set_param_future_.valid() ||
                set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
            set_param_future_ = detector_param_client_->set_parameters(
                    {param}, [this, param](const ResultFuturePtr &results) {
                        for (const auto &result: results.get()) {
                            if (!result.successful) {
                                RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                                return;
                            }
                        }
                        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
                        initial_set_param_ = true;
                    });
        }


    }
}
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav_client::NavClient)