#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('six_point_control')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('start')
        arm.go()
        rospy.sleep(2)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        # 共六个点
        target_pose1 = PoseStamped()
        target_pose1.header.frame_id = reference_frame
        target_pose1.header.stamp = rospy.Time.now()     
        target_pose1.pose.position.x = -0.20954
        target_pose1.pose.position.y = 0.089024
        target_pose1.pose.position.z = 0.60222
        target_pose1.pose.orientation.x = 0.37667
        target_pose1.pose.orientation.y = 0.75248
        target_pose1.pose.orientation.z = 0.4445
        target_pose1.pose.orientation.w = -0.30711
        
        target_pose2 = PoseStamped()
        target_pose2.header.frame_id = reference_frame
        target_pose2.header.stamp = rospy.Time.now()     
        target_pose2.pose.position.x = -0.1128
        target_pose2.pose.position.y =  -0.14707
        target_pose2.pose.position.z = 0.37064
        target_pose2.pose.orientation.x = -0.39081
        target_pose2.pose.orientation.y = 0.83395
        target_pose2.pose.orientation.z = -0.38933
        target_pose2.pose.orientation.w = -0.01492

        target_pose3 = PoseStamped()
        target_pose3.header.frame_id = reference_frame
        target_pose3.header.stamp = rospy.Time.now()     
        target_pose3.pose.position.x = 0.026495
        target_pose3.pose.position.y = -0.33704
        target_pose3.pose.position.z = 0.36236
        target_pose3.pose.orientation.x = -0.30971
        target_pose3.pose.orientation.y = -0.19614
        target_pose3.pose.orientation.z = -0.22526
        target_pose3.pose.orientation.w = 0.9027

        target_pose4 = PoseStamped()
        target_pose4.header.frame_id = reference_frame
        target_pose4.header.stamp = rospy.Time.now()     
        target_pose4.pose.position.x = -0.38729
        target_pose4.pose.position.y = -0.16604
        target_pose4.pose.position.z = 0.41109
        target_pose4.pose.orientation.x = 0.45951
        target_pose4.pose.orientation.y = -0.17333
        target_pose4.pose.orientation.z = 0.77484
        target_pose4.pose.orientation.w = -0.39803

        target_pose5 = PoseStamped()
        target_pose5.header.frame_id = reference_frame
        target_pose5.header.stamp = rospy.Time.now()     
        target_pose5.pose.position.x = 0.13853
        target_pose5.pose.position.y = 0.36664
        target_pose5.pose.position.z = 0.3988
        target_pose5.pose.orientation.x = -0.20134
        target_pose5.pose.orientation.y = -0.0048734
        target_pose5.pose.orientation.z = 0.58476
        target_pose5.pose.orientation.w = 0.78581

        target_pose6 = PoseStamped()
        target_pose6.header.frame_id = reference_frame
        target_pose6.header.stamp = rospy.Time.now()     
        target_pose6.pose.position.x = -0.42208
        target_pose6.pose.position.y = -0.010391
        target_pose6.pose.position.z = 0.37943
        target_pose6.pose.orientation.x = 0.39368
        target_pose6.pose.orientation.y = -0.1043
        target_pose6.pose.orientation.z = 0.91279
        target_pose6.pose.orientation.w = 0.031039
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        #1
        arm.set_pose_target(target_pose1, end_effector_link)
        arm.go()
        rospy.loginfo("成功到达第一个位置"+
        "\ntarget_pose1.pose.position.x = -0.20954"+
        "\ntarget_pose1.pose.position.y = 0.089024"+
        "\ntarget_pose1.pose.position.z = 0.60222"+
        "\ntarget_pose1.pose.orientation.x = 0.37667"+
        "\ntarget_pose1.pose.orientation.y = 0.75248"+
        "\ntarget_pose1.pose.orientation.z = 0.4445"+
        "\ntarget_pose1.pose.orientation.w = -0.30711")
        rospy.sleep(1)
         
        #2
        arm.set_pose_target(target_pose2, end_effector_link)
        arm.go()
        rospy.loginfo("成功到达第二个位置"+
        "\ntarget_pose2.pose.position.x = -0.1128"+
        "\ntarget_pose2.pose.position.y =  -0.14707"+
        "\ntarget_pose2.pose.position.z = 0.37064"+
        "\ntarget_pose2.pose.orientation.x = -0.39081"+
        "\ntarget_pose2.pose.orientation.y = 0.83395"+
        "\ntarget_pose2.pose.orientation.z = -0.38933"+
        "\ntarget_pose2.pose.orientation.w = -0.01492")
        rospy.sleep(1)

        #3
        arm.set_pose_target(target_pose3, end_effector_link)
        arm.go()
        rospy.loginfo("成功到达第三个位置"+
        "\ntarget_pose3.pose.position.x = 0.026495"+
        "\ntarget_pose3.pose.position.y = -0.33704"+
        "\ntarget_pose3.pose.position.z = 0.36236"+
        "\ntarget_pose3.pose.orientation.x = -0.30971"+
        "\ntarget_pose3.pose.orientation.y = -0.19614"+
        "\ntarget_pose3.pose.orientation.z = -0.22526"+
        "\ntarget_pose3.pose.orientation.w = 0.9027")
        rospy.sleep(1)
        
        #4
        arm.set_pose_target(target_pose4, end_effector_link)
        arm.go()
        rospy.loginfo("成功到达第四个位置"+        
        "\ntarget_pose4.pose.position.x = -0.38729"+
        "\ntarget_pose4.pose.position.y = -0.16604"+
        "\ntarget_pose4.pose.position.z = 0.41109"+
        "\ntarget_pose4.pose.orientation.x = 0.45951"+
        "\ntarget_pose4.pose.orientation.y = -0.17333"+
        "\ntarget_pose4.pose.orientation.z = 0.77484"+
        "\ntarget_pose4.pose.orientation.w = -0.39803")
        rospy.sleep(1)
        
        #5
        arm.set_pose_target(target_pose5, end_effector_link)
        arm.go()
        rospy.loginfo("成功到达第五个位置"+        
        "\ntarget_pose5.pose.position.x = 0.13853"+
        "\ntarget_pose5.pose.position.y = 0.36664"+
        "\ntarget_pose5.pose.position.z = 0.3988"+
        "\ntarget_pose5.pose.orientation.x = -0.20134"+
        "\ntarget_pose5.pose.orientation.y = -0.0048734"+
        "\ntarget_pose5.pose.orientation.z = 0.58476"+
        "\ntarget_pose5.pose.orientation.w = 0.78581")
        rospy.sleep(1)
        
        #6
        arm.set_pose_target(target_pose6, end_effector_link)
        arm.go()
        rospy.loginfo("成功到达第六个位置"+        
        "\ntarget_pose6.pose.position.x = -0.42208"+
        "\ntarget_pose6.pose.position.y = -0.010391"+
        "\ntarget_pose6.pose.position.z = 0.37943"+
        "\ntarget_pose6.pose.orientation.x = 0.39368"+
        "\ntarget_pose6.pose.orientation.y = -0.1043"+
        "\ntarget_pose6.pose.orientation.z = 0.91279"+
        "\ntarget_pose6.pose.orientation.w = 0.031039")
        rospy.sleep(1)
        
        # 控制机械臂回到初始化位置
        arm.set_named_target('start')
        arm.go()
        rospy.loginfo("点位控制任务完成!")

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
