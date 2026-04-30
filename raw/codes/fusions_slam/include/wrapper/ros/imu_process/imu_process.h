/*
 * @Author: ylh 
 * @Date: 2024-04-08 21:25:13 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-04-08 22:36:04
 */
#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H
#include "fusion_slam/type/imu_type.h"
#include "sensor_msgs/Imu.h"

class ImuProcess{
    private:
    public:
    using Ptr = std::shared_ptr<ImuProcess>;
    ImuProcess();
    ~ImuProcess();
    int process(const sensor_msgs::ImuPtr &msg, ImuType& outMsg);
};

#endif