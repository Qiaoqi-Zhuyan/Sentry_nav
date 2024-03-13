//
// Created by shuxy on 24-3-10.
//

#ifndef BUILD_ICP_LOCATION_NODE_HPP
#define BUILD_ICP_LOCATION_NODE_HPP

#include <cmath>
#include <vector>
#include <chrono>
#include <iostream>
#include <deque>
#include <mutex>
#include <typeinfo>
#include <thread>

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/msg/odometry.hpp"

//#include "roborts_msgs/Relocate.h"
//#include "roborts_msgs/LocationInfo.h"

// tf2
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// pcl
//PCD文件输入输出操作
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"

#include "pcl/point_cloud.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace icp_localization{
    class ScanIcpLocationNode : rclcpp::Node {
    public:
        explicit ScanIcpLocationNode(const rclcpp::NodeOptions &options);
        ~ScanIcpLocationNode();
    private:

        void ScanCallBakc(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laserScan_msg);

        void MapCallBack(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &map_msg);


    };




}


#endif //BUILD_ICP_LOCATION_NODE_HPP
