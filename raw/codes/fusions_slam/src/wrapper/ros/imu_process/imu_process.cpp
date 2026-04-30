/*
 * @Author: ylh 
 * @Date: 2024-04-08 22:33:12 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-04-08 22:37:34
 */
#include "wrapper/ros/imu_process/imu_process.h"

int ImuProcess::process(const sensor_msgs::ImuPtr &msg, ImuType& outMsg){
        outMsg.timeStamp.fromNsec(msg->header.stamp.toNSec());
        outMsg.acceleration= {msg->linear_acceleration.x, msg->linear_acceleration.y,
                            msg->linear_acceleration.z};
        outMsg.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        return 0;
    }