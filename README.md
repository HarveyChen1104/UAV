# PX4入门
本工作空间包含三个PX4仿真入门实验：指点飞行画矩形，地面站给定航点飞行和识别aruco二维码降落

无人机gazebo仿真 = 实飞程序 + mavros + posix_sitl.launch

posix_sitl.launch = PX4 + world.launch + spawn_model

gazebo: .world, .model, plugin 

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
