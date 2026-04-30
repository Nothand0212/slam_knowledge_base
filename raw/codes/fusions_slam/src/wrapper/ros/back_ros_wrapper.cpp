/*
 * @Author: ylh 
 * @Date: 2026-01-06 22:03:31 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2026-01-06 23:08:25
 */

#include "wrapper/ros/back_ros_wrapper.h"

BackWrapper::BackWrapper(ros::NodeHandle &nh, const std::string& cfgPath){
    YAML::Node configNode;
    lioTopic = "/lio_odometry";
    rtkTopic = "/gps_odometry";
    configNode = YAML::LoadFile(cfgPath);
    rtkTopic = configNode["wrapper"]["rtk_topic"].as<std::string>();
    lioTopic = configNode["wrapper"]["lio_topic"].as<std::string>();
    lioGpsOptPtr = std::make_shared<LioGpsOpt>();
    lioOdometrySubscriber = nh.subscribe(lioTopic, 100000, &BackWrapper::lioOdometryMsgCallBack, this);
    gpsOdometrySubscriber = nh.subscribe(rtkTopic, 100000, &BackWrapper::rtkOdomMsgCallBack, this);
    backPathPub = nh.advertise<nav_msgs::Path>("/back_path", 10);
    backOdomPub = nh.advertise<nav_msgs::Odometry>("back_odom", 100);
    LOG(INFO) <<"\033[1;32m"<< "back_wrapper: " << "\033[0m" << std::endl;
    std::cout << "      lio_topic           : " << lioTopic << std::endl;
    std::cout << "      rtk_topic   : " << rtkTopic << std::endl;
    run();
}

BackWrapper::~BackWrapper(){}

void BackWrapper::publishMsg(){
    Pose backPose;
    lioGpsOptPtr->getGlobalPose(backPose);
    static nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp.fromSec(backPose.timeStamp.sec());
    geometry_msgs::PoseStamped psd;
    psd.pose.position.x = backPose.position[0];
    psd.pose.position.y = backPose.position[1];
    psd.pose.position.z = backPose.position[2];
    // LOG(INFO) << "pose " << psd.pose.position.x << " " << psd.pose.position.y << " " << psd.pose.position.z;
    path.poses.emplace_back(psd);
    backPathPub.publish(path);
    nav_msgs::Odometry odom;
    odom.header.stamp.fromSec(backPose.timeStamp.sec());
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = backPose.position[0];
    odom.pose.pose.position.y = backPose.position[1];
    odom.pose.pose.position.z = backPose.position[2];
    odom.pose.pose.orientation.x = backPose.rotation.x();
    odom.pose.pose.orientation.y = backPose.rotation.y();
    odom.pose.pose.orientation.z = backPose.rotation.z();
    odom.pose.pose.orientation.w = backPose.rotation.w();
    backOdomPub.publish(odom);
}

void BackWrapper::run(){
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        publishMsg();
        rate.sleep();
    }
}

void BackWrapper::lioOdometryMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg){
     Pose lioPose;
     lioPose.timeStamp.fromSec(msg->header.stamp.toSec());
     lioPose.position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
     lioPose.rotation.x() = msg->pose.pose.orientation.x;
     lioPose.rotation.y() = msg->pose.pose.orientation.y;
     lioPose.rotation.z() = msg->pose.pose.orientation.z;
     lioPose.rotation.w() = msg->pose.pose.orientation.w;
    //  LOG(INFO) <<" lio_odom call_back " << lioPose.timeStamp.sec() <<" "<< lioPose.position;
     lioGpsOptPtr->inLio(lioPose);
}

void BackWrapper::rtkOdomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg){
       Pose rtkPose;
       rtkPose.timeStamp.fromSec(msg->header.stamp.toSec());
       rtkPose.position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
       rtkPose.rotation.x() = msg->pose.pose.orientation.x;
       rtkPose.rotation.y() = msg->pose.pose.orientation.y;
       rtkPose.rotation.z() = msg->pose.pose.orientation.z;
       rtkPose.rotation.w() = msg->pose.pose.orientation.w;
    //    LOG(INFO) <<" rtk_odom call_back " << rtkPose.timeStamp.sec() << " " << rtkPose.position;
       lioGpsOptPtr->inGpsOdom(rtkPose);
}
