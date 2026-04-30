/*
 * @Author: ylh 
 * @Date: 2024-09-13 21:39:31 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-13 22:00:12
 */

#include "fusion_slam/modules/map/map_engine.h"

MapEngine::MapEngine(const std::string& configPath){
    mapPath = "";
    loadMapSucceed = false;
    globalMapPtr = pcl::make_shared<PCLPointCloud>();
    kdtreePtr = pcl::make_shared<KDTree>();
    YAML::Node configNode;
    configNode = YAML::LoadFile(configPath);
    mapPath = configNode["map_engine"]["map_path"].as<std::string>();
    LOG(INFO) <<"\033[1;32m"<< "map_engine: " << "\033[0m" << std::endl;
    std::cout << "      map_path        : " << mapPath << std::endl;
}

MapEngine::~MapEngine(){}

int MapEngine::loadMap(){
    if (pcl::io::loadPCDFile<PointType>(mapPath, *globalMapPtr) == -1) {
        LOG(INFO) << " load map failed ! path : " << mapPath;
        return -1;
    }
    LOG(INFO) << " load map success ! path : " << mapPath;
    loadMapSucceed = true;
    kdtreePtr->setInputCloud(globalMapPtr);
    return 0;
}

PCLPointCloudConstPtr MapEngine::getMap(){
    return globalMapPtr;
}

KDTreeConstPtr MapEngine::readKDtree(){
    return kdtreePtr;
}