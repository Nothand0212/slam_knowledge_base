/*
 * @Author: ylh 
 * @Date: 2026-01-06 21:46:48 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2026-01-06 23:06:14
 */

#ifndef BACK_WRAPPER_H
#define BACK_WRAPPER_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "fusion_slam/type/pose.h"
#include "fusion_slam/modules/back/lio_gps_opt.h"


class BackWrapper{
    private:
        LioGpsOpt::Ptr lioGpsOptPtr;
        std::string lioTopic, rtkTopic;
        ros::Subscriber lioOdometrySubscriber;
        ros::Subscriber gpsOdometrySubscriber;
        ros::Publisher backPathPub;
        ros::Publisher backOdomPub;
        void lioOdometryMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg);
        void rtkOdomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg);
        void publishMsg();
        
    public:
        BackWrapper(ros::NodeHandle &nh, const std::string& cfgPath);
        ~BackWrapper();
        void run();    
};

#endif