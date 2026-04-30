/*
 * @Author: ylh 
 * @Date: 2024-04-07 22:09:20 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-13 22:12:29
 */

#ifndef _FUSION_SLAM_ROS_WRAPPER_H_
#define _FUSION_SLAM_ROS_WRAPPER_H_
#include <ctime>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "fusion_slam/modules/location/location.h"
#include "wrapper/ros/imu_process/imu_process.h"
#include "wrapper/ros/lidar_process/velodyne_process.h"
#include "wrapper/ros/lidar_process/ouster_process.h"
#include "fusion_slam/modules/sensors/rtk/rtk_switch.h"
#include <yaml-cpp/yaml.h>
#include "novatel_msgs/INSPVAX.h" // 注意安装novatel_msgs消息库

constexpr time_t GPS_EPOCH_UNIX = 315964800;
class LocationWrapper
{
    private:
        Location::Ptr locationPtr;
        ImuProcess::Ptr imuProcessPtr;
        RtkSwitch::Ptr rtkSwitchPtr;
        VelodyneProcess::Ptr velodynelidarProcessPtr;
        OusterProcess::Ptr ousterlidarProcessPtr;
        ros::Subscriber cloudSubscriber;
        ros::Subscriber imuSubscriber;
        ros::Subscriber rtkInspvaxSubscriber;
        ros::Subscriber rtkNavsatSubscriber;
        ros::Subscriber rtkOdomSubscriber;
        ros::Publisher currCloudPub;
        ros::Publisher pathPub;
        ros::Publisher rtkPathPub;
        ros::Publisher localMapPub;
        
        void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg);
        void imuMsgCallBack(const sensor_msgs::ImuPtr &msg);
        void rtkInspvaxMsgCallBack(const novatel_msgs::INSPVAX::ConstPtr& msg);
        void rtkNavsatMsgCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void rtkOdomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg);
        void run();
        void publishMsg();
        Eigen::Matrix4d T_imu_lidar;
        Eigen::Matrix4d T_imu_ant;
        Eigen::Matrix<double,4,1> lidarXYBox;
        int lidarType;
        int rtkDateType;
        int lidarLineNum;
        int pointGapNum;
        bool useRtk;
        bool savePose;
        std::string rtkPosePath;
        std::string curPosePath;
        std::string cfgPath;

    public:
        LocationWrapper(ros::NodeHandle &nh, const std::string& cfgPath);
        ~LocationWrapper();
        int init(const std::string& path);
        time_t gps2unix(const int& gpsWeek, const double& gpsTow);
        bool gpsInitSuccess;
};

#endif

