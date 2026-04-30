/*
 * @Author: ylh 
 * @Date: 2024-08-19 21:43:47 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-08-19 22:49:15
 */

#ifndef IMU_STATIC_INIT_H
#define IMU_STATIC_INIT_H
#include "fusion_slam/type/imu_type.h"
#include "fusion_slam/type/base_type.h"

class ImuStaticInit{
private:
        bool _isStaticalInited;
        Eigen::Vector3d _staticalAcc;
        Eigen::Vector3d _staticalGyro;
        Eigen::Vector3d _gravity;
        double _imuScale;
        double _imuInitDuration;
        int _imuDataNum = 0;
        double _startTime = 0.0;
        bool _firstImuData = true;
        
    public:
        using Ptr = std::shared_ptr<ImuStaticInit>;
        ImuStaticInit(const std::string& path);
        int addImu(const ImuType& imuData);
        int getStaticalImuParams(Eigen::Vector3d& acc, Eigen::Vector3d& gyro, Eigen::Vector3d& gravity, double& scale);
};

#endif