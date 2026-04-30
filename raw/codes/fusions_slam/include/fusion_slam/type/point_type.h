/*
 * @Author: ylh 
 * @Date: 2024-04-20 15:54:13 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-04-20 16:49:50
 */
#ifndef POINT_TYPE_H
#define POINT_TYPE_H
#include <pcl/impl/pcl_base.hpp>
#include <pcl/pcl_base.h>
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct EIGEN_ALIGN16 PointType {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t offset_time;
    std::int32_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)   // 纳秒
    (std::int32_t, ring, ring)
)
#endif