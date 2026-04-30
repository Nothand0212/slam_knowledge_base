/*
 * @Author: ylh 
 * @Date: 2024-09-13 15:30:04 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-13 21:59:38
 */

#ifndef MAP_ENGINE_H
#define MAP_ENGINE_H
#include "fusion_slam/type/pointcloud.h"
#include "fusion_slam/type/base_type.h"
#include "fusion_slam/math/math.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

class MapEngine{
    private:
        bool loadMapSucceed;
        PCLPointCloudPtr globalMapPtr;
        KDTreePtr kdtreePtr;
        std::string mapPath;
    public:
        using Ptr = std::shared_ptr<MapEngine>;
        MapEngine(const std::string& configPath);
        ~MapEngine();

        int loadMap();
        PCLPointCloudConstPtr getMap();
        KDTreeConstPtr readKDtree();
};
#endif