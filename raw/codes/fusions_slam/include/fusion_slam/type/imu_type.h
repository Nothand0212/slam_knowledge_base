/*
 * @Author: ylh 
 * @Date: 2024-04-07 22:20:47 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-04-08 21:55:21
 */

#ifndef IMU_TYPE_H
#define IMU_TYPE_H

#include <Eigen/Dense>
#include "fusion_slam/type/timestamp.h"
class ImuType
{
public:
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();
    TimeStamp timeStamp;
    void clear(){
        acceleration = Eigen::Vector3d::Zero();
        gyroscope = Eigen::Vector3d::Zero();
        timeStamp = 0;
    }
    ImuType operator + (const ImuType& imu){
        ImuType res;
        res.acceleration = this->acceleration+imu.acceleration;
        res.gyroscope = this->gyroscope+imu.gyroscope;
        return res;
    }
    ImuType operator *(double k){
        ImuType res;
        res.acceleration = this->acceleration *k;
        res.gyroscope = this->gyroscope *k;
        return res;
    }
    ImuType  operator/(double k){
        ImuType res;
        res.acceleration = this->acceleration /k;
        res.gyroscope = this->gyroscope /k;
        return res;
    }
    friend std::ostream & operator<<(std::ostream& ostream,const ImuType& imu){
        ostream<<"imu_time: "<<imu.timeStamp.sec()<<" s | imu_acc: "<<imu.acceleration.transpose()<<" | imu_gro: "<< imu.gyroscope.transpose() ;
        return ostream;
    }
};
#endif