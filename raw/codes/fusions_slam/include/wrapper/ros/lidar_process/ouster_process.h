/*
 * @Author: ylh 
 * @Date: 2024-04-21 22:00:58 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-04-21 22:32:36
 */
#ifndef OUSTOR_PROCESS_H
#define OUSTOR_PROCESS_H
#include <sensor_msgs/PointCloud2.h>
#include "fusion_slam/type/base_type.h"
#include "pcl_conversions/pcl_conversions.h"
#include <glog/logging.h>

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class OusterProcess
{
private:
    int _lidarLineNum = 32;
    int _pointGapNum = -1;
public:
    int init(const int& lidarLineNum, const int& pointGapNum){
        _lidarLineNum = lidarLineNum;
        _pointGapNum = pointGapNum;
        return 0;
    }
    using Ptr = std::shared_ptr<OusterProcess>;
    bool process(const sensor_msgs::PointCloud2Ptr &msg, PointCloud &cloud){
        pcl::PointCloud<ouster_ros::Point> rs_cloud;
        pcl::fromROSMsg(*msg,rs_cloud);
        cloud.cloudPtr->clear();
        // double end_time = msg->header.stamp.toSec();
        // double start_time = end_time + rs_cloud[0].t;//第一个点时间戳
        double startTime = msg->header.stamp.toSec();
        int i = 0;
        for (auto &&p : rs_cloud)
        {   
            if(_lidarLineNum >32){
                if(_lidarLineNum == 64){
                    if(p.ring % 2 != 0){
                        continue;
                    }
                }else if(_lidarLineNum == 128){
                    if(p.ring % 4 != 0){
                        continue;
                    }
                }else{
                    LOG(ERROR) << "lidarLineNum error";
                    continue;
                }
            }
            if(_pointGapNum > 0){
                ++i;
                if(i % _pointGapNum != 0){
                    continue;
                }
            }
            PointType point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            // point.intensity = p.reflectivity;
            point.intensity = p.intensity;
            point.offset_time = p.t;//计算时间偏差
            // point.ring = p.ring;
            point.ring = 0;
            cloud.cloudPtr->emplace_back(point);
            // std::cout << p.t << std::endl;
        }
        // cloud.timeStamp.fromSec(start_time);
        cloud.timeStamp.fromSec(startTime);
        return true;
    }
};
#endif