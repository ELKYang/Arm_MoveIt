# Arm_MoveIt
Simulation operation of six-axis manipulator based on ROS (including point position control and trajectory control of the manipulator)
# 机械臂操作文档

## 一.平台说明

操作系统:`Ubuntu 16.04`

仿真平台:`ROS+RVIZ+MoveIt`

机械臂:六轴两爪七自由度机械臂

## 二.操作说明

- 确保已在`Ubuntu16.04`环境下安装`ROS kinetic`,并安装`MoveIt`.

- 将*Elk_Arm*文件夹下的三个功能包*elk_arm_description,elk_arm_planning,elk_moveit_config*放到工作空间目录的*src*文件夹下.

- 在工作空间目录下执行

  ```xml-dtd
  $ catkin_make
  ```

  进行功能包编译.

- 功能实现

  新建终端,执行	

  ```xml-dtd
  $ roslaunch elk_moveit_config demo.launch
  ```

  可看到如下RVIZ界面

  ![1](C:\Users\Mr_Ya\Desktop\操作文档.assets\1.png)

  1. 点位控制

     新建终端,执行

     ```xml-dtd
     $ rosrun elk_arm_planning six_point_control.py 
     ```

     可观察机械臂到达指定的六个点位并在终端中有显示.

  2. 椭圆轨迹运动

     新建终端,执行

     ```xml-dtd
     $ rosrun elk_arm_planning oval_trajectory_planning.py 
     ```

     可观察机械臂绘制出椭圆轨迹.如下

     <img src="C:\Users\Mr_Ya\Desktop/2.png" alt="2" style="zoom: 50%;" />

  3. "8"字型轨迹运动

     新建终端,执行

     ```xml-dtd
     $ rosrun elk_arm_planning eight_trajectory_planning.py
     ```

     可观察机械臂绘制出"8"字型轨迹.如下

     <img src="C:\Users\Mr_Ya\Desktop/3.png" alt="3" style="zoom:50%;" />

  4. 避障轨迹规划

     新建终端,执行

     ```xml-dtd
     $ rosrun elk_arm_planning obstacles_planning.py
     ```

     可观察机器人完成避障轨迹规划.如下

     <img src="C:\Users\Mr_Ya\Desktop/4.png" alt="4" style="zoom:50%;" />

- 注:在RVIZ中出现尾迹重叠看不清楚的情况,可将RVIZ左边栏中的displays-->robot_model--->links--->grasping_frame--->show trail取消勾选后再次勾选,重新执行操作命令即可.

## 三.机械臂说明

- 机械臂结构

   *elk_arm_description/urdf/elk_arm.xacro*文件为机械臂的描述文件,其中定义了机械臂的外观,物理属性(碰撞属性等),关节最大旋转角度,关节最大旋转速度等,机械臂的DH参数表如下所示

  | 关节 | 连杆长度 | 连杆扭角 | 关节偏距 | 关节转角 |
  | ---- | -------- | -------- | -------- | -------- |
  | 1    | 0        | 0        | 0        | theta 1  |
  | 2    | 0        | -90      | 0.1      | theta 2  |
  | 3    | 0.14     | 0        | 0.14     | theta 3  |
  | 4    | 0        | -90      | 0.22     | theta 4  |
  | 5    | 0        | 90       | 0.06     | theta 5  |
  | 6    | 0        | -90      | 0.06     | theta 6  |

- MoveIt机械臂控制平台

  - MoveIt系统架构

    MoveIt通过move_group节点进行机械臂的路径规划,具体结构如下

    ![5](C:\Users\Mr_Ya\Desktop\操作文档.assets\5.png)

  - MoveIt运动规划器的结构

    ![6](C:\Users\Mr_Ya\Desktop\操作文档.assets\6.png)

  - 程序中使用*ik_track*运动规划器,进行正逆运动学的求解,但由于此运动规划器得到的是数值解而非解析解,会导致频繁出现相同位置规划路径不同的问题.

## 四.代码实现说明

- 点位控制(six_point_control.py)

  使用关节空间控制机械臂到达空间中的六个点位

  ```python
  target_pose1.pose.position.x = -0.20954
  target_pose1.pose.position.y = 0.089024
  target_pose1.pose.position.z = 0.60222
  target_pose1.pose.orientation.x = 0.37667
  target_pose1.pose.orientation.y = 0.75248
  target_pose1.pose.orientation.z = 0.4445
  target_pose1.pose.orientation.w = -0.30711
  ```

  position为机械臂末端坐标,orientation为机械臂末端姿态的四元数表示.

- 椭圆轨迹控制(oval_trajectory_planning.py)

  微元法将椭圆轨迹分成很多份的直线笛卡尔轨迹的规划

  ```python
  centerA = target_pose.pose.position.y
  centerB = target_pose.pose.position.z
  long_axis = 0.03
  short_axis=0.06
  for th in numpy.arange(0, 6.28, 0.005):
      target_pose.pose.position.y = centerA + long_axis * math.cos(th)-0.03
      target_pose.pose.position.z = centerB + short_axis * math.sin(th)
      wpose = deepcopy(target_pose.pose)
      waypoints.append(deepcopy(wpose))
  ```

  将椭圆分出的点位加入列表中,遍历列表即可完成椭圆轨迹.

- "8"字型轨迹控制

  同椭圆轨迹规划,使用微元法,将"8"字看做两个圆的拼接

  ```python
  for th in numpy.arange(0, 6.284, 0.005):
      target_pose.pose.position.y = centerA + radius * math.cos(th-1.571)
      target_pose.pose.position.z = centerB + radius * math.sin(th-1.571)+0.04
      wpose = deepcopy(target_pose.pose)
      waypoints.append(deepcopy(wpose))
  for th in numpy.arange(0, 6.284, 0.005):
      target_pose.pose.position.y = centerA + radius * math.cos(-th+1.571)
      target_pose.pose.position.z = centerB + radius * math.sin(-th+1.571)-0.04
      wpose = deepcopy(target_pose.pose)
      waypoints.append(deepcopy(wpose)) 
  ```

- 避障轨迹规划

  在环境中加入桌子及方块等障碍物,然后利用MoveIt进行运动规划,即可实现MoveIt控制机械臂自主避障的功能,自主避障为MoveIt中本身自带的功能,无需再进行操作.

  
