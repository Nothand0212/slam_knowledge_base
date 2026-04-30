/*
 * @Author: ylh 
 * @Date: 2024-04-17 22:28:54 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-30 22:55:22
 */

#ifndef PROPAGATE_H
#define PROPAGATE_H
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fusion_slam/type/pose.h"
#include "fusion_slam/type/imu_type.h"
#include "fusion_slam/type/pointcloud.h"
#include <yaml-cpp/yaml.h>
#include "fusion_slam/modules/ieskf/ieskf.h"
#include "fusion_slam/type/measure_group.h"
#include <vector>

class Propagate{
    private:
        struct IMUPose6d {  // 参考fastlio自定义消息
        double time;
        Eigen::Vector3d acc;
        Eigen::Vector3d angvel;
        Eigen::Vector3d vel;
        Eigen::Vector3d pos;
        Eigen::Quaterniond rot;
        IMUPose6d(double time_ = 0, Eigen::Vector3d a_ = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d av_ = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d v_ = Eigen::Vector3d::Zero(),
                    Eigen::Vector3d p_ = Eigen::Vector3d::Zero(),
                    Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity()) {
                time = time_;
                acc = a_;
                angvel = av_;
                vel = v_;
                pos = p_;
                rot = q_;
            }
        };
        Eigen::Vector3d lastAcc;
        Eigen::Vector3d lastAngvel;
        Eigen::Vector3d rtkTransNoise;
        Eigen::Vector3d rtkRpyNoise;
        Eigen::Vector3d velNoise;
        int rtkType;
    public:
        using Ptr = std::shared_ptr<Propagate>;
        double imuAccScale;
        ImuType lastImu;
        double lastPcdEndTime;
        Propagate(const std::string &configPath);
        ~Propagate();
        int run(MeasureGroupAdd &msg, IESKF::Ptr &ieskfPtr);
    
};

#endif