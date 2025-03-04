# PX4入门
本工作空间包含三个PX4仿真入门实验：指点飞行画矩形，地面站给定航点飞行和识别aruco二维码降落

gazebo: .world, .model, plugin 

world: 环境信息&module信息

model: .urdf .sdf

plugin: 模型绑定ROS，实现传感器输出，电机控制输入

无人机gazebo仿真 = 实飞程序 + mavros + posix_sitl.launch

posix_sitl.launch = PX4 + world.launch + spawn_model

## 系统环境
Ubuntu 20.04

ROS Noetic

PX4 V1.13

## 指点行画矩形
在offnode功能包中，
```bash
roslaunch px4 mavros_posix_sitl.launch
rosrun offnode offnode_node
```

**当PX4使用offboard模式时，应当先进offboard模式再解锁。** 若解锁后未及时进入 Offboard 模式，飞控可能默认进入其他模式（如 Position 模式），导致控制权混乱。

## 给定航点飞行
在offnode功能包中，
```bash
roslaunch px4 mavros_posix_sitl.launch
rosrun offnode wayPointMission.py
```
**当PX4使用mission模式时，应当先解锁再切mission。** Mission 模式依赖预设的航点任务，无需实时外部控制。解锁后，无人机默认进入稳定模式（如 Position 模式），操作者可先确保飞行环境安全，再手动切换到 Mission 模式。

## 识别aruco二维码降落
在aruco_detect_land功能包中，
```bash
# 导航到其下的shell文件夹，运行.sh脚本文件。脚本文件中的绝对地址要改成自己电脑中的相应地址。
bash aruco_land_gazebo.sh
```
这里使用的方法是直接向PX4传输aruco二维码在世界坐标系中的位置。
当然也尝试了自己写个PID，输出结果给到PX4传输速度指令中。但假如因为曝光或者晃动的原因，aruco二维码没识别到，UAV会保持之前的速度，可能会飞出二维码识别范围，导致系统发散。

相机模型使用的是PX4中自带的fpv_cam。通过话题去读取相机的内参，发现他的内参矩阵cx，cy参数是有问题的。
实验结果是依据它提供的内参矩阵去识别aruco二维码，XY两个轴的数据有很大的误差。
然后本人通过单目相机小孔成像模型的原理去尝试，应该把它提供的cx,cy两个内参参数除以2，实验结果也是很好的能够使aruco二维码识别估计其位置准确。

mavros作为连接飞控端和ros端的程序，有这么四个坐标系，即飞控端的body系和local系及ros端的body系和local系：fcu_body,fcu_local,ros_body,ros_local。fcu坐标是系是NED，ROS坐标系是ENU，fuc_body坐标系又叫aircraft坐标系，ros_body坐标系又叫base_link坐标系。
详见[基于mavros源码详解px4-mavros中的ENU NED FLU FRD坐标系及位姿转换](https://zhuanlan.zhihu.com/p/20734057891)

在本程序中，mavros_msgs::PositionTarget 消息类型的frame设置为FRAME_LOCAL_NED,但我们在set Z值是还是给正值，因为mavros会自动转换坐标系。

