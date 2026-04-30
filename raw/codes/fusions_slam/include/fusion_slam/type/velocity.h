/*
 * @Author: ylh 
 * @Date: 2024-05-05 14:51:27 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-05 14:56:35
 */
#ifndef VELOCITY_H
#define VELOCITY_H
#include <Eigen/Dense>
#include "timestamp.h"
struct Velocity{
    TimeStamp timeStamp;
    Eigen::Vector3d velocity;
};
#endif