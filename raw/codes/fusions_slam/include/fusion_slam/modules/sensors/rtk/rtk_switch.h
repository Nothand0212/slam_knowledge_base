/*
 * @Author: ylh 
 * @Date: 2024-05-01 11:57:11 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-01 12:05:24
 */
#ifndef RTK_SWITCH_H
#define RTK_SWITCH_H
#include "fusion_slam/type/pose.h"

class RtkSwitch
{
    private:
        Pose fristPose;
        bool fristRtk;
        double a, b, e2, f; //地球参数, 长半轴,短半轴,偏心率,扁率
    public:
        using Ptr = std::shared_ptr<RtkSwitch>;
        RtkSwitch();
        ~RtkSwitch();
        int LLA2Enu(const Eigen::Vector3d& lla, const Eigen::Vector3d& refLla, Eigen::Vector3d& enu);
        int LLA2Ecef(const Eigen::Vector3d& lla, Eigen::Vector3d& ecef);
        int ecef2Enu(const Eigen::Vector3d& ecef, const Eigen::Vector3d& refLla, const Eigen::Vector3d& refEcef, Eigen::Vector3d& enu);
        int LLA2Enu2(const Eigen::Vector3d& lla, const Eigen::Vector3d& ref_lla, Eigen::Vector3d& enu);
};
#endif