////
//// Created by shuxy on 24-3-14.
////
//
//#include "nav_client/vision_client_node.hpp"
//
//
//#include <tf2/LinearMath/Quaternion.h>
//
//#include <rclcpp/logging.hpp>
//#include <rclcpp/qos.hpp>
//#include <rclcpp/utilities.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//
//// C++ system
//#include <cstdint>
//#include <functional>
//#include <map>
//#include <memory>
//#include <string>
//#include <vector>
//
//
//namespace nav_client
//{
//    VisionClientNode::VisionClientNode(const rclcpp::NodeOptions & options)
//            : Node("rm_serial_driver", options)
//    {
//        RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
//
//        getParams();
//
//        // TF broadcaster
//        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
//        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
//
//        // Create Publisher
//        latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
//        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
//
//        // Detect parameter client
//        detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
//
//        // Tracker reset service client
//        reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");
//
//        try {
//
//            client_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
//            if (client_socket_fd < 0)
//                RCLCPP_ERROR(this->get_logger(), "Error create socket fd");
//            else
//                RCLCPP_INFO(this->get_logger(), "create socket fd");
//
//            serv_port_ = declare_parameter("serv_port", 51718);
//            serv_ip_ = declare_parameter("serv_ip", "192.168.123.110");
//
//            serv_addr_.sin_family = AF_INET;
//            serv_addr_.sin_addr.s_addr = inet_addr(serv_ip_.c_str());
//            serv_addr_.sin_port = htons(serv_port_);
//
//            RCLCPP_INFO(get_logger(), "send to %s:%d", serv_ip_.c_str(), serv_port_);
//
//            receive_thread_ = std::thread(&VisionClientNode::receiveData, this);
//        } catch (const std::exception & ex) {
//            RCLCPP_ERROR(this->get_logger(), "Socket Error: %s", ex.what());
//            throw ex;
//        }
//
//        aiming_point_.header.frame_id = "odom";
//        aiming_point_.ns = "aiming_point";
//        aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
//        aiming_point_.action = visualization_msgs::msg::Marker::ADD;
//        aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
//        aiming_point_.color.r = 1.0;
//        aiming_point_.color.g = 1.0;
//        aiming_point_.color.b = 1.0;
//        aiming_point_.color.a = 1.0;
//        aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
//
//        // Create Subscription
//        target_sub_ = this->create_subscription<nav_auto_aim_interfaces::msg::Target>(
//                "/tracker/target", rclcpp::SensorDataQoS(),
//                std::bind(&VisionClientNode::sendData, this, std::placeholders::_1));
//    }
//
//    VisionClientNode::~VisionClientNode()
//    {
//        close(client_socket_fd);
//    }
//
//    void VisionClientNode::receiveData(){
//
//        while (rclcpp::ok()) {
//            try {
//
//                bzero(buffer_, BUFFER_SIZE);
//                socklen_t serv_addr_len = sizeof (serv_addr_);
//                int n_recvfrom = recvfrom(client_socket_fd, buffer_, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&serv_addr_, &serv_addr_len);
//
//                if (n_recvfrom < 0)
//                    RCLCPP_ERROR(this->get_logger(), "Error reading from server");
//                else
//                    RCLCPP_INFO(this->get_logger(), "Reading data %ld from server %s", ++recv_data_cunt_, serv_ip_.c_str());
//
//                ReceivePacket recvPacket;
//                unpack(recvPacket, buffer_);
//
//                serial_driver_->port()->receive(header);
//
//                if (header[0] == 0x5A) {
//                    data.resize(sizeof(ReceivePacket) - 1);
//                    serial_driver_->port()->receive(data);
//
//                    data.insert(data.begin(), header[0]);
//                    ReceivePacket packet = fromVector(data);
//
//                    bool crc_ok =
//                            crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
//                    if (crc_ok) {
//                        if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
//                            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
//                            previous_receive_color_ = packet.detect_color;
//                        }
//
//                        if (packet.reset_tracker) {
//                            resetTracker();
//                        }
//
//                        geometry_msgs::msg::TransformStamped t;
//                        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
//                        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
//                        t.header.frame_id = "odom";
//                        t.child_frame_id = "gimbal_link";
//                        tf2::Quaternion q;
//                        q.setRPY(packet.roll, packet.pitch, packet.yaw);
//                        t.transform.rotation = tf2::toMsg(q);
//                        tf_broadcaster_->sendTransform(t);
//
//                        if (abs(packet.aim_x) > 0.01) {
//                            aiming_point_.header.stamp = this->now();
//                            aiming_point_.pose.position.x = packet.aim_x;
//                            aiming_point_.pose.position.y = packet.aim_y;
//                            aiming_point_.pose.position.z = packet.aim_z;
//                            marker_pub_->publish(aiming_point_);
//                        }
//                    } else {
//                        RCLCPP_ERROR(get_logger(), "CRC error!");
//                    }
//                } else {
//                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
//                }
//            } catch (const std::exception & ex) {
//                RCLCPP_ERROR_THROTTLE(
//                        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
//                reopenPort();
//            }
//        }
//    }
//
//    void VisionClientNode::sendData(const nav_auto_aim_interfaces::msg::Target::SharedPtr msg)
//    {
//        const static std::map<std::string, uint8_t> id_unit8_map{
//                {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
//                {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};
//
//        try {
//            SendPacket packet;
//            packet.tracking = msg->tracking;
//            packet.id = id_unit8_map.at(msg->id);
//            packet.armors_num = msg->armors_num;
//            packet.x = msg->position.x;
//            packet.y = msg->position.y;
//            packet.z = msg->position.z;
//            packet.yaw = msg->yaw;
//            packet.vx = msg->velocity.x;
//            packet.vy = msg->velocity.y;
//            packet.vz = msg->velocity.z;
//            packet.v_yaw = msg->v_yaw;
//            packet.r1 = msg->radius_1;
//            packet.r2 = msg->radius_2;
//            packet.dz = msg->dz;
//            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
//
//            std::vector<uint8_t> data = toVector(packet);
//
//            serial_driver_->port()->send(data);
//
//            std_msgs::msg::Float64 latency;
//            latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
//            RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
//            latency_pub_->publish(latency);
//        } catch (const std::exception & ex) {
//            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
//            reopenPort();
//        }
//    }
//
//    void VisionClientNode::reopenPort()
//    {
//        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
//        try {
//            if (serial_driver_->port()->is_open()) {
//                serial_driver_->port()->close();
//            }
//            serial_driver_->port()->open();
//            RCLCPP_INFO(get_logger(), "Successfully reopened port");
//        } catch (const std::exception & ex) {
//            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
//            if (rclcpp::ok()) {
//                rclcpp::sleep_for(std::chrono::seconds(1));
//                reopenPort();
//            }
//        }
//    }
//
//    void VisionClientNode::setParam(const rclcpp::Parameter & param)
//    {
//        if (!detector_param_client_->service_is_ready()) {
//            RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
//            return;
//        }
//
//        if (
//                !set_param_future_.valid() ||
//                set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
//            RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
//            set_param_future_ = detector_param_client_->set_parameters(
//                    {param}, [this, param](const ResultFuturePtr & results) {
//                        for (const auto & result : results.get()) {
//                            if (!result.successful) {
//                                RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
//                                return;
//                            }
//                        }
//                        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
//                        initial_set_param_ = true;
//                    });
//        }
//    }
//
//    void VisionClientNode::resetTracker()
//    {
//        if (!reset_tracker_client_->service_is_ready()) {
//            RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
//            return;
//        }
//
//        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//        reset_tracker_client_->async_send_request(request);
//        RCLCPP_INFO(get_logger(), "Reset tracker!");
//    }
//
//}  // namespace rm_serial_driver
//
//#include "rclcpp_components/register_node_macro.hpp"
//
//// Register the component with class_loader.
//// This acts as a sort of entry point, allowing the component to be discoverable when its library
//// is being loaded into a running process.
//RCLCPP_COMPONENTS_REGISTER_NODE(nav_client::VisionClientNode)
//
