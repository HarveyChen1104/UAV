#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <yaml-cpp/yaml.h>

cv::Mat frame;  //初始化frame时不指定分辨率和类型，这样保证程序可以接受任意分辨率的图片，以及rgb图或者灰度图。  
void cam_image_cb(const sensor_msgs::Image::ConstPtr& msg){
    try{
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        frame = cv_ptr->image;

        // ROS_INFO("Received a %d x %d image", frame.cols, frame.rows);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "aruco_det_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    int dictionary_id;
    int aruco_id;
    float aruco_length;
    std::string camera_param_path;

    nh1.param<int>("dictionary_id", dictionary_id, 11);
    nh1.param<int>("aruco_id", aruco_id, 27);
    nh1.param<float>("aruco_length", aruco_length, 0.2);
    nh1.param<std::string>("camera_param_path", camera_param_path, "/home/harvey/Documents/my_mavros_ws/src/aruco_detect_land/config/camera_gazebo.yaml");


    ros::Subscriber cam_sub = nh.subscribe<sensor_msgs::Image>("/iris/usb_cam/image_raw", 1, cam_image_cb);
    ros::Publisher aruco_det_image_pub = nh.advertise<sensor_msgs::Image>("/aruco/det_image", 10);
    ros::Publisher aruco_det_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/aruco/pose", 10);

    // 创建一个CvBridge对象
    cv_bridge::CvImage cv_image;

    // 定义Aruco字典和参数
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // 加载 YAML 文件
    YAML::Node config = YAML::LoadFile(camera_param_path);
    double fx = config["fx"].as<double>();
    double fy = config["fy"].as<double>();
    double cx = config["cx"].as<double>();
    double cy = config["cy"].as<double>();

    double k1 = config["k1"].as<double>();
    double k2 = config["k2"].as<double>();
    double p1 = config["p1"].as<double>();
    double p2 = config["p2"].as<double>();
    double k3 = config["k3"].as<double>();

    cv::Mat camera_InnerMatrix = (cv::Mat_<double>(3,3) << 
                                    fx,  0, cx,
                                    0,  fy, cy,
                                    0,   0,  1);

    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);


    ros::Rate rate(10.0);
    while(ros::ok()){

        if(!frame.empty()){
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // opencv已经有现成的aruco二维码检测函数了，分为了两个步骤，先提取角点，再根据二维码角点估计相对位姿

            // 检测aruco二维码
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            // aruco二维码角点检测
            cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

            // 绘制检测结果
            if (markerIds.size() > 0){
                cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

                // 估计姿态 pose = position + attitude
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(markerCorners, aruco_length, camera_InnerMatrix, distCoeffs, rvecs, tvecs);

                for(int i=0; i<markerIds.size(); ++i){
                    cv::aruco::drawAxis(frame, camera_InnerMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                }

                auto it = std::find(markerIds.begin(), markerIds.end(), aruco_id);
                if(it != markerIds.end()){
                    int index = std::distance(markerIds.begin(), it);
                    std::cout << "aruco_id: " << aruco_id << "found at index: " << index << std::endl;
                    std::cout << "aruco_detect: " << "X " << tvecs[index][0] << "Y " << tvecs[index][1] << "Z " << tvecs[index][2] << std::endl;
                    geometry_msgs::PoseStamped aruco_detect_pose;
                    aruco_detect_pose.pose.position.x = tvecs[index][0];
                    aruco_detect_pose.pose.position.y = tvecs[index][1];
                    aruco_detect_pose.pose.position.z = tvecs[index][2];

                    cv::Mat rotation_vector = (cv::Mat_<double>(3,1) << rvecs[index][0], rvecs[index][1], rvecs[index][2]);
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rotation_vector, rotation_matrix);

                    // 旋转矩阵转为四元数
                    double w = std::sqrt(1 + rotation_matrix.at<double>(0, 0) + rotation_matrix.at<double>(1, 1) + rotation_matrix.at<double>(2, 2)) / 2;
                    double x = (rotation_matrix.at<double>(2, 1) - rotation_matrix.at<double>(1, 2)) / (4 * w);
                    double y = (rotation_matrix.at<double>(0, 2) - rotation_matrix.at<double>(2, 0)) / (4 * w);
                    double z = (rotation_matrix.at<double>(1, 0) - rotation_matrix.at<double>(0, 1)) / (4 * w);

                    aruco_detect_pose.pose.orientation.x = x;
                    aruco_detect_pose.pose.orientation.y = y;
                    aruco_detect_pose.pose.orientation.z = z;
                    aruco_detect_pose.pose.orientation.w = w;

                    aruco_detect_pose.header.stamp = ros::Time::now();
                    aruco_det_pose_pub.publish(aruco_detect_pose);  
                }
                else{
                    std::cout << "aruco_id: " << aruco_id << "not found in the vector markerIds." << std::endl;
                }
            }

            cv_image.image = frame;
            cv_image.encoding = "bgr8";
            sensor_msgs::ImagePtr img_msg = cv_image.toImageMsg();

            aruco_det_image_pub.publish(img_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
}


