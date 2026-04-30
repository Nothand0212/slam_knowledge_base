/*
 * @Author: ylh 
 * @Date: 2024-04-07 22:20:56 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-01 13:03:53
 */

#ifndef MEASURE_GROUP_H
#define MEASURE_GROUP_H

#include "fusion_slam/type/imu_type.h"
#include "fusion_slam/type/pointcloud.h"
#include "fusion_slam/type/pose.h"
#include <deque>
#include <map>
#include <cstdint>

struct DataUnit{
    int type; // 0 imu 1 lidar 2 gps 3 camera 4 speed
    ImuType imu;
    PointCloud cloud;
    Pose rtkPose;
};

struct MeasureGroupAdd{
    uint64_t lidarBeginTime;
    std::map<uint64_t, DataUnit> mapDatas;
    uint64_t lidarEndTime;
};


#endif

