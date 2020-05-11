#!/usr/bin/env python
# -*- coding: utf-8 -*-



# 控制规划组的确定
# 设置目标位姿
# 设置运动约束(可选)
# 使用moveit!规划一条可到达的目标轨迹
# 修改轨迹
# 执行轨迹
import rospy,sys
import actionlib
import moveit_commander 

#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        #moveit_commander.roscpp_initialize(sys.argv)
        #arm = moveit_commander.MoveGroupCommander('test1')
        
        # # 是否需要回到初始化的位置
        # reset = rospy.get_param('~reset', False)
        
        # # 机械臂中joint的命名
        # arm_joints = ['joint1',
        #               'joint2',
        #               'joint3', 
        #               'joint4',
        #               'joint5',]
        
        # if reset:
        #     arm_goal  = [0, 0, 0, 0, 0, 0]
        # else:
        #     arm_goal  = [-0.3, -1.0, 0.5, 0.8, 1.0, -0.7]
    
        # rospy.loginfo('Waiting for arm trajectory controller...')       
        # arm_client = actionlib.SimpleActionClient('learm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # arm_client.wait_for_server()        
        # rospy.loginfo('...connected.')  
        # print(arm_goal)
    
        # # 创建轨迹数据
        # arm_trajectory = JointTrajectory()
        # arm_trajectory.joint_names = arm_joints
        # arm_trajectory.points.append(JointTrajectoryPoint())
        # arm_trajectory.points[0].positions = arm_goal
        # arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        # arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        # arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # rospy.loginfo('Moving the arm to goal position...')
        
        # # 创建一个轨迹目标的空对象
        # arm_goal = FollowJointTrajectoryGoal()
        # arm_goal.trajectory = arm_trajectory
        # arm_goal.goal_time_tolerance = rospy.Duration(0.0)
        # arm_client.send_goal(arm_goal)

        # # 等待机械臂运动结束
        # arm_client.wait_for_result(rospy.Duration(5.0))
        # rospy.loginfo('...done')
        
if __name__ == '__main__': 
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
