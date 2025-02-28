#include "ros/ros.h"

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double position[3] = {0, 0, 0};
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    position[2] = msg->pose.position.z;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    float HEIGHT;
    float SIDE;
    nh1.param<float>("height", HEIGHT, 2);
    nh1.param<float>("side", SIDE, 1.5);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber postion_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = HEIGHT;

    for (int i = 0; i < 100; i++){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool offboard_flag = 0;
    bool land_flag = 0;
    bool arm_flag = 0;
    int task_flag = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(current_state.mode !="OFFBOARD" && (ros::Time::now()-last_request>ros::Duration(5.0)) && !offboard_flag){
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                offboard_flag = 1; 
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else{
            if(!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0)) && !arm_flag){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    arm_flag = 1;
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    
        if(!land_flag && offboard_flag && arm_flag){
            if( (abs(position[0]-pose.pose.position.x)<0.2 && abs(position[1]-pose.pose.position.y)<0.2 && abs(position[2]-pose.pose.position.z)<0.2) && (ros::Time::now()-last_request>ros::Duration(5.0)) ){
                task_flag += 1;
                ROS_INFO("Next task%d", task_flag);
                last_request = ros::Time::now(); 
            } 
            if(task_flag==1){
                pose.pose.position.x = SIDE;
                pose.pose.position.y = 0;
                pose.pose.position.z = HEIGHT;
            }
            else if(task_flag==2){
                pose.pose.position.x = SIDE;
                pose.pose.position.y = SIDE;
                pose.pose.position.z = HEIGHT; 
            }
            else if(task_flag==3){
                pose.pose.position.x = 0;
                pose.pose.position.y = SIDE;
                pose.pose.position.z = HEIGHT; 
            }
            else if(task_flag==4){
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = HEIGHT; 
            }
            else if(task_flag==5){
                set_mode_client.call(land_set_mode);
                if(land_set_mode.response.mode_sent){
                    land_flag = 1;
                    ROS_INFO("land enabled");   
                }  
            }
        }


        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    
}

