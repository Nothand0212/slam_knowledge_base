/*
 * @Author: ylh 
 * @Date: 2024-04-09 22:26:09 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-04 22:53:57
 */

#ifndef LIO_GPS_OPT_H
#define LIO_GPS_OPT_H
#include <mutex>
#include <string>
#include <yaml-cpp/yaml.h>
#include "fusion_slam/type/pose.h"
#include "fusion_slam/type/base_type.h"
#include "fusion_slam/modules/back/factors.h"

struct MeasurePose{
    double ts;
    Pose lio;
    Pose gps;
};

class LioGpsOpt{
    private:
        std::mutex mLioPose, mGpsPose, mLioTsGpsPose;
        std::map<double, Pose> lioPoseMap, globalLioPoseMap, gpsPoseMap, lioTsGpsPoseMap;
        std::deque<Pose> lioPoseBuf, gpsPoseBuf, lioTsGpsPoseBuf;
        Pose lastPose;
        bool hasNewLioPose, hasNewGpsPose, hasNewGpsOdom;
        std::thread threadOpt;
        std::thread threadAlign;
        Eigen::Vector3d interpPosition(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double alpha);
        Eigen::Quaterniond interpQuaternion(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double alpha);
        void optimize();
        bool alignPose(const double &ts, Pose &out);//align gpsPoseMap by lio time
        bool syncMeasurePose(MeasurePose &msg);
        Eigen::Matrix4d WGPS_T_WLIO;

    public:
        using Ptr = std::shared_ptr<LioGpsOpt>;
        LioGpsOpt();
        ~LioGpsOpt();
        void inLio(const Pose &in);
        void inGpsOdom(const Pose &in);
        void getGlobalPose(Pose &out);
};


#endif
