#!/bin/sh
gnome-terminal \
  --window -e 'bash -c " \
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/harvey/Documents/my_mavros_ws/src/aruco_detect_land/models; \
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH://home/harvey/Documents/my_mavros_ws/src/aruco_detect_land/models_for_worlds; \
    source ~/Documents/my_mavros_ws/devel/setup.bash; \
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware; \
    roslaunch aruco_detect_land mavros_posix_sitl_aruco_land.launch; \
    exec bash" ' \
  --tab -e 'bash -c "sleep 5; \
    source ~/Documents/my_mavros_ws/devel/setup.bash; \
    roslaunch aruco_detect_land aruco_detect.launch; \
    exec bash" ' \
  --tab -e 'bash -c "sleep 5; \
    source ~/Documents/my_mavros_ws/devel/setup.bash; \
    rosrun aruco_detect_land aruco_land; \
    exec bash" ' \
  --window -e 'bash -c "sleep 10; \
    rostopic echo /mavros/local_position/pose; \
    exec bash" ' \
  --window -e 'bash -c "sleep 10; \
    rostopic echo /aruco/pose; \
    exec bash" ' \
  --window -e 'bash -c "sleep 10; \
    rostopic echo /mavros/setpoint_raw/local; \
    exec bash"'
