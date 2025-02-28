# PX4入门
本工作空间包含三个PX4仿真入门实验：指点飞行画矩形，地面站给定航点飞行和识别aruco二维码降落

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

## 给定航点飞行
在offnode功能包中，
```bash
roslaunch px4 mavros_posix_sitl.launch
rosrun offnode wayPointMission.py
```

## 识别aruco二维码降落
在aruco_detect_land功能包中，
```bash
# 导航到其下的shell文件夹，运行.sh脚本文件。脚本文件中的绝对地址要改成自己电脑中的相应地址。
bash aruco_land_gazebo.sh
```
这里使用的方法是直接向PX4传输aruco二维码在世界坐标系中的位置。
当然也尝试了自己写个PID，输出结果给到PX4传输速度指令中。但假如因为曝光或者晃动的原因，aruco二维码没识别到，UAV会保持之前的速度，可能会飞出二维码识别范围，导致系统发散。
