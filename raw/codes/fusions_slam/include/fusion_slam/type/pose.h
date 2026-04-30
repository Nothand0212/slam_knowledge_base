#ifndef POSE_H
#define POSE_H
#include <Eigen/Dense>
#include "timestamp.h"
struct Pose
{
    TimeStamp timeStamp;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d position;
};
struct Rot{
    TimeStamp timeStamp;
    Eigen::Quaterniond rotation;
};

struct Pos{
    TimeStamp timeStamp;
    Eigen::Vector3d position;
};

#endif
