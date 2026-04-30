/*
 * @Author: ylh 
 * @Date: 2024-04-08 21:11:55 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-04-20 16:01:47
 */

#ifndef VELODYNE_PROCESS_H
#define VELODYNE_PROCESS_H
#include <sensor_msgs/PointCloud2.h>
#include "fusion_slam/type/base_type.h"
#include "pcl_conversions/pcl_conversions.h"
#include <glog/logging.h>

namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time,time)
    (std::uint16_t, ring, ring)
)
class VelodyneProcess
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
    using Ptr = std::shared_ptr<VelodyneProcess>;
    bool process(const sensor_msgs::PointCloud2Ptr &msg, PointCloud &cloud){
        pcl::PointCloud<velodyne_ros::Point> rs_cloud;
        pcl::fromROSMsg(*msg,rs_cloud);
        cloud.cloudPtr->clear();
        double endTime = msg->header.stamp.toSec();
        double startTime = endTime + rs_cloud[0].time;//第一个点时间戳
        int i = 0;
        for (auto &&p : rs_cloud)
        {
            // if(p.ring % 4 != 0) continue; // 128线 下采样到 32线
            // // ++i;
            // // if(i % 5 != 0) continue;
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
            double pointTime = p.time +endTime;
            PointType point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.intensity = p.intensity;
            point.offset_time = (pointTime - startTime)*1e9;//计算时间偏差
            point.ring = p.ring;
            cloud.cloudPtr->emplace_back(point);
        
        }
        cloud.timeStamp.fromSec(startTime);
        return true;
    }
};
#endif