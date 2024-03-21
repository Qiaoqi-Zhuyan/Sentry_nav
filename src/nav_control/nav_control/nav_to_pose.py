# python

from enum import Enum

# nav2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ros2
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

# robot
import robot_state

nav = BasicNavigator()


class RobotDecision(Enum):
    pass

def setInitialPose(x, y, z, w):
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = x
    init_pose.pose.position.y = y
    init_pose.pose.position.z = z
    init_pose.pose.position.w = w
    return init_pose

def setGoalPose(x, y, w):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.w = w

    return goal_pose

# 移动到定点
def navToPoseCallback(init_pose, goal_pose):
    nav.setInitialPose(init_pose)
    nav.lifecycleStartup()
    nav.goToPose(goal_pose)
    i = 0
    while not nav.isTaskComplete():
        i = i + 1
        feedback = nav.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                nav.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                nav.goToPose(goal_pose)

    return nav.getResult()

# class GoToPoseActionServer(Node):
#     def __init__(self):
#         super.__init__("navigation_action_server")
#         self._action_server_ = ActionServer(
#         )
#
# class GameSatutsClient(Node):
#



def main(args=None):
    rclpy.init(args=args)

    init_pose = setInitialPose(0.0, 0.0, 0.0, 0.0)
    goal_pose = setGoalPose(0.0, 0.0, 0.0)

    minimal_subscriber = ()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()







