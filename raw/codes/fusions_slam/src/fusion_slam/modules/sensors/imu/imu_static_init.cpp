/*
 * @Author: ylh 
 * @Date: 2024-08-19 21:44:29 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-08-19 22:58:36
 */

#include "fusion_slam/modules/sensors/imu/imu_static_init.h"
#include <yaml-cpp/yaml.h> 

ImuStaticInit::ImuStaticInit(const std::string& path){
    _isStaticalInited = false;
    _staticalAcc = Eigen::Vector3d::Zero();
    _staticalGyro = Eigen::Vector3d::Zero();
    _gravity = Eigen::Vector3d::Zero();
    _imuScale = 1.0;
    _imuInitDuration = 1.0;
    _imuDataNum = 0;
    _startTime = 0.0;
    _firstImuData = true;
    YAML::Node config = YAML::LoadFile(path);
    _imuInitDuration = config["front"]["imu_init_duration"].as<double>();
}

int ImuStaticInit::addImu(const ImuType& imuData){
    _staticalAcc += imuData.acceleration;
    _staticalGyro += imuData.gyroscope;
    ++_imuDataNum;
    return 0;
}

int ImuStaticInit::getStaticalImuParams(Eigen::Vector3d& acc, Eigen::Vector3d& gyro, Eigen::Vector3d& gravity, double& scale){
    _staticalAcc /= double(_imuDataNum);
    _staticalGyro /= double(_imuDataNum);
    _imuScale = GRAVITY / _staticalAcc.norm();
    _gravity = -_staticalAcc / _staticalAcc.norm() * GRAVITY;
    _staticalAcc += _gravity;

    acc = _staticalAcc;
    gyro = _staticalGyro;
    gravity = _gravity;
    scale = _imuScale;
    return 0;
}