# python
import os
from enum import Enum

# nav2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ros2
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node

# robot
from nav_interfaces.msg import GameStatus
from auto_aim_interfaces.msg import Target


'''
逻辑:

导航决策:
订阅比赛状态:
    接受比赛开始的信号
        移动到定点
            1. 启动导航程序
            2. 选择目的地
            3. 发送 cmd_vel
            4. 到达目的的结束导航
            5, 启动扫描程序
        移动到多个点
            1. 启动导航程序
            2. 启动route程序
            3. 云台扫描

订阅自瞄
    当相机自瞄没有数据时：
        1. 启动扫描程序
        2. 启动导航程序*
    当相机有数据的时候
        1. 启动自瞄程序
        2. 关闭导航任务
        3. 关闭导航程序
        

'''

# cmds=(  "ros2 launch nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav localization:=amcl"
#         )
#
#
# for cmd in "${cmds[@]}"
#     do
# echo Current CMD : "$cmd"
# gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
# sleep 0.2
# done



class RobotDecision(Enum):
    NAV_TO_POSE = 0
    NAV_THROUGH_POSE = 1
    SCAN = 2

class RobotMode(Enum):
    NAV = 0
    AUTO_AIM = 1

shell_script = ("cd ~/mapping/simulation/socket_rv ;"
                "bash start_auto_aim.sh")

class GameStatusSubscriber(Node):
    def __init__(self):
        super().__init__('game_status_sub')
        self.subscription = self.create_subscription(
            GameStatus,
            "/game_status",
            self.navStartCallback,
            10,
        )

    def navStartCallback(self, game_status : GameStatus):
        nav = BasicNavigator()

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = nav.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.001
        init_pose.pose.position.y = 0.002
        init_pose.pose.orientation.z = 0.004
        init_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 2.990
        goal_pose.pose.position.y = 0.1600
        goal_pose.pose.orientation.w = 0.7026


        if game_status.game_state == 1:
            print("start navToPose......")

            nav.setInitialPose(init_pose)
            nav.lifecycleStartup()
            nav.goToPose(goal_pose)
            i = 0
            while not nav.isTaskComplete():
                i = i + 1
                feedback = nav.getFeedback()
                if feedback and i % 5 == 0:
                    # print(
                    #     'Estimated time of arrival: '
                    #     + '{0:.0f}'.format(
                    #         Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    #         / 1e9
                    #     )
                    #     + ' seconds.'
                    # )

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                        n = os.system(shell_script)
                        print(n)
                        nav.cancelTask()
                        nav.lifecycleShutdown()
                    #
                    # # Some navigation request change to demo preemption
                    # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                    #     goal_pose.pose.position.x = -3.0
                    #     nav.goToPose(goal_pose)

            if nav.isTaskComplete():
                n = os.system(shell_script)
                print(n)
                nav.lifecycleShutdown()


            result = nav.getResult()

            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')



def main(args=None):
    rclpy.init()

    subscriber = GameStatusSubscriber()
    rclpy.spin(subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()







