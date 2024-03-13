# python
from enum import Enum

# nav2
from nav2_simple_commander.robot_navigator import BasicNavigator

# ros2
import rclpy
from geometry_msgs.msg import PoseStamped

# robot
import robot_state




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
class RobotDecision(Enum):
    pass


navigator = BasicNavigator()

def setGoalPose():
    pass

# 移动到定点
def navTopose(pose_stamped : PoseStamped, goal_pose : PoseStamped):
    pass

def main():
    pass





