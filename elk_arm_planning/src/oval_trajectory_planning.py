#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy

import math
import numpy

class MoveItCircleDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('oval_trajectory_planning', anonymous=True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.3)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂先回到初始化位置
        arm.set_named_target('start')
        arm.go()
        rospy.sleep(1)
                                               
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()    
        target_pose.pose.position.x = 0.38665
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.37226
        target_pose.pose.orientation.x = 1
        target_pose.pose.orientation.y = 0.00007
        target_pose.pose.orientation.z = -0.0024684
        target_pose.pose.orientation.w = 0.00003

        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()

        # 初始化路点列表
        waypoints = []
                
        # 将圆弧上的路径点加入列表
        waypoints.append(target_pose.pose)

        centerA = target_pose.pose.position.y
        centerB = target_pose.pose.position.z
        long_axis = 0.03
        short_axis=0.06

        for th in numpy.arange(0, 6.28, 0.005):
            target_pose.pose.position.y = centerA + long_axis * math.cos(th)-0.03
            target_pose.pose.position.z = centerB + short_axis * math.sin(th)
            wpose = deepcopy(target_pose.pose)
            waypoints.append(deepcopy(wpose))

            #print('%f, %f' % (Y, Z))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)
        rospy.loginfo("位姿控制---->椭圆轨迹规划 成功!")
        # 控制机械臂先回到初始化位置
        arm.set_named_target('start')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCircleDemo()
    except rospy.ROSInterruptException:
        pass