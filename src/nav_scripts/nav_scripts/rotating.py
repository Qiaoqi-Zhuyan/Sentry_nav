#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = rclpy.create_node("rotate_robot")
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    #rate = node.create_rate(10)

    twist = Twist()
    twist.angular.z = 5.0

    while rclpy.ok():
        publisher.publish(twist)
        node.get_logger().info("rotating")
        #rate.sleep()
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

