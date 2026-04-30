#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include "fusion_slam/type/point_type.h"
#include "fusion_slam/type/timestamp.h"

using PCLPointCloud = pcl::PointCloud<PointType>;
using PCLPointCloudPtr = PCLPointCloud::Ptr;
using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;
struct PointCloud{
    using Ptr = std::shared_ptr<PointCloud>;
    TimeStamp timeStamp;
    PCLPointCloudPtr cloudPtr;
    PointCloud(){
        cloudPtr = pcl::make_shared<PCLPointCloud>();
    }
};

#endif

