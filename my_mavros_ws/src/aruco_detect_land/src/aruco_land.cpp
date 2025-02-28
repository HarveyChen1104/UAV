#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

using namespace std;

mavros_msgs::State current_state;

Eigen::Isometry3d Twa, Tca, Tbc, Twb, Twc;
Eigen::Matrix3d rotation_matrix_bc, rotation_matrix_wb, rotation_matrix_ca;
Eigen::Vector3d t_bc, t_wb, t_ca, t_wa;
Eigen::Quaterniond q_wb, q_ca;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    t_wb << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    q_wb = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    rotation_matrix_wb = q_wb.toRotationMatrix();

    Twb.linear() = rotation_matrix_wb;
    Twb.translation() = t_wb;
}

void aruco_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    t_ca << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    q_ca = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    rotation_matrix_ca = q_ca.toRotationMatrix();

    Tca.linear() = rotation_matrix_ca;
    Tca.translation() = t_ca;
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "aruco_land_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    bool AUTO_ARM_OFFBOARD;
    nh1.param<bool>("auto_arm_offborad", AUTO_ARM_OFFBOARD, true);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber aruco_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, aruco_pos_cb);

    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    t_bc << 0, 0, 0;
    rotation_matrix_bc << 0, -1,  0,
                         -1,  0,  0,
                          0,  0, -1;
    Tbc.linear() = rotation_matrix_bc;
    Tbc.translation() = t_bc;
    t_ca << 0, 0, 0;
    
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b100111111000;    // 100 111 111 000
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    pos_setpoint.position.z = 2;
    pos_setpoint.yaw = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i>0; --i){
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool offboard_flag = false;
    bool land_flag = false;
    int state_flag = 0;

    ros::Time time_snap;
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(AUTO_ARM_OFFBOARD){
            if(current_state.mode!="OFFBOARD" && (ros::Time::now()-last_request>ros::Duration(3.0)) && !offboard_flag){ // 意味着offboard在程序运行时，只能切入一次
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    offboard_flag = true;
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else if(!current_state.armed && (ros::Time::now()-last_request>ros::Duration(3.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        else{

        }

        //ros::Duration(1.0).sleep();
        if((abs(t_wb[0])<0.2 && abs(t_wb[1])<0.2 && abs(t_wb[2]-2)<0.2) && (ros::Time::now()-last_request > ros::Duration(5.0))){
            state_flag = 1;
            // Twa = Twb * Tbc * Tca;
            // t_wa = Twa.translation();
            // std::cout << "Aruco land: X: " << t_wa[0] << "Y: " << t_wa[1] << "Z: " << t_wa[2] << endl;
            last_request = ros::Time::now();
            std::cout << "state 1" << std::endl;
        }
        else if( state_flag == 1){
            pos_setpoint.position.x = 2.5;
            pos_setpoint.position.y = 2.5;
            pos_setpoint.position.z = 2; 
            if((abs(t_wb[0]-pos_setpoint.position.x)<0.2 && (abs(t_wb[1]-pos_setpoint.position.y)<0.2) && abs(t_wb[2]-pos_setpoint.position.z)<0.2) && (t_ca[2] != 0)){
                state_flag = 2;
                last_request = ros::Time::now();
                std::cout << "state 2" << std::endl;
            }
        }
        else if( state_flag == 2){
            Twa = Twb * Tbc * Tca;
    
            // 获取T中的平移向量
            t_wa = Twa.translation();
            std::cout << "Aruco land: X: " << t_wa[0] << "Y: " << t_wa[1] << "Z: " << t_wa[2]+2 << endl;
            pos_setpoint.position.x = t_wa[0];
            pos_setpoint.position.y = t_wa[1];
            pos_setpoint.position.z = t_wa[2] + 2; 

            // std::cout << "X: " << pos_setpoint.position.x << "Y: " << pos_setpoint.position.y << "Z: " << pos_setpoint.position.z << endl;
            if((abs(t_ca[0])<0.1)&&(abs(t_ca[1])<0.1)){
                state_flag = 3;
                time_snap = ros::Time::now();
                std::cout << "state 3" << std::endl;
            }
        }
        else if( state_flag == 3){
            ros::Time now_time = ros::Time::now();
            double now_time_relate_from_state2 = (now_time-time_snap).toSec();


            /***
            pos_setpoint.type_mask = 0b100111000100;
            pos_setpoint.coordinate_frame = 1;
            pos_setpoint.velocity.x = 0;
            pos_setpoint.velocity.y = 0;
            pos_setpoint.velocity.z = -0.1;
            pos_setpoint.position.x = aruco_position_ENU[0];
            pos_setpoint.position.y = aruco_position_ENU[1];
            pos_setpoint.yaw = 0;
            ***/

            //Twc = Twb * Tbc ;
            Twa = Twb * Tbc * Tca ;
            // 获取T中的平移向量
            t_wa = Twa.translation();

            pos_setpoint.position.x = t_wa[0];
            pos_setpoint.position.y = t_wa[1];
            pos_setpoint.position.z = t_wa[2] + 2 - 0.1*now_time_relate_from_state2; 

            if(t_wb[2] < 0.15){
                state_flag = 4;
            }
        }
        else if( state_flag == 4){
            std::cout << "state 4" << std::endl;

            Twa = Twb * Tbc * Tca;
            // 获取T中的平移向量
            t_wa = Twa.translation();

            pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
            pos_setpoint.coordinate_frame = 1;

            pos_setpoint.position.x = t_wa[0];
            pos_setpoint.position.y = t_wa[1];
            pos_setpoint.position.z = t_wa[2] + 0.1; 
            pos_setpoint.yaw = 0;
        
            if((abs(t_ca[0])<0.1)&&(abs(t_ca[1])<0.1)){
                state_flag = 5;
            } 
        }
        else if( state_flag == 5){
            if( current_state.mode != "AUTO.LAND" ){
                set_mode_client.call(land_set_mode);
                if(land_set_mode.response.mode_sent){
                    ROS_INFO("land enabled");
                }  
            }
        }

        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }
}

