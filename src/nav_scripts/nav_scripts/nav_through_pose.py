from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():

    rclpy.init()

    navigator = BasicNavigator()

    # set init pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.001
    initial_pose.pose.position.y = 0.002
    initial_pose.pose.orientation.z = 0.004
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)


    navigator.waitUntilNav2Active()

    goal_poses = []

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 6.5
    goal_pose1.pose.position.y = 0.12
    goal_pose1.pose.orientation.w = 0.0




