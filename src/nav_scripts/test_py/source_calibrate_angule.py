#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
from math import radians, copysign
from tf2_geometry_msgs import QuaternionStamped
from tf2_ros import TransformListener
from math import pi

class CalibrateAngular(Node):
    def __init__(self):
        super().__init__('calibrate_angular')

        # How fast will we check the odometry values?
        self.rate = 10
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        # The test angle is 360 degrees
        self.test_angle = 2 * pi

        self.speed = 0.1  # radians per second
        self.tolerance = 1  # degrees converted to radians
        self.odom_angular_scale_correction = 1
        self.start_test = True

        # Publisher to control the robot's speed
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 5)

        # The base frame is usually base_link or base_footprint
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # The odom frame is usually just /odom
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        # Initialize the tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Bring up rqt_reconfigure to control the test.")

    def timer_callback(self):
        if self.start_test:
            # Get the current rotation angle from tf
            self.odom_angle = self.get_odom_angle()
            self.get_logger().info("self.odom_angle: " + str(self.odom_angle))

            last_angle = self.odom_angle
            turn_angle = 0
            self.test_angle *= -1

            error = self.test_angle - turn_angle
            self.get_logger().info("error: " + str(error))

            while abs(error) > self.tolerance and self.start_test:
                move_cmd = Twist()
                move_cmd.angular.z = copysign(self.speed, error)
                self.cmd_vel.publish(move_cmd)

                self.odom_angle = self.get_odom_angle()
                self.get_logger().info("current rotation angle: " + str(self.odom_angle))

                delta_angle = self.odom_angular_scale_correction * (self.odom_angle - last_angle)
                self.get_logger().info("delta_angle: " + str(delta_angle))

                turn_angle += abs(delta_angle)
                self.get_logger().info("turn_angle: " + str(turn_angle))

                error = self.test_angle - turn_angle
                self.get_logger().info("error: " + str(error))

                last_angle = self.odom_angle

            self.cmd_vel.publish(Twist())

            self.start_test = False

    def get_odom_angle(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame,
                                                        rclpy.time.Time(),
                                                        rclpy.duration.Duration(seconds=1.0))
            quat_msg = QuaternionStamped()
            quat_msg.header.stamp = transform.header.stamp
            quat_msg.transform.rotation = transform.transform.rotation

            euler = tf2_ros.transformations.euler_from_quaternion([quat_msg.transform.rotation.x,
                                                                   quat_msg.transform.rotation.y,
                                                                   quat_msg.transform.rotation.z,
                                                                   quat_msg.transform.rotation.w])

            return euler[2]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("TF Exception")

    def shutdown(self):
        # Always stop the robot when shutting down the node
        self.get_logger().info("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rclpy.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    calibrate_angular = CalibrateAngular()
    rclpy.spin(calibrate_angular)
    calibrate_angular.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()