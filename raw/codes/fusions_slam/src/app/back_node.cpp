/*
 * @Author: ylh 
 * @Date: 2026-01-06 21:14:26 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2026-01-06 23:14:40
 */

#include "wrapper/ros/back_ros_wrapper.h"

int main(int argc, char **argv)
{
    if(argc<2){
        LOG(ERROR)<<"Usage: ./fusion_slam_back_node [config.yaml]";
        return -1;
    }
    std::string cfgPath = argv[1];
    ros::init(argc,argv,"fusion_slam_back_node");
    ros::NodeHandle nh;
    std::shared_ptr<BackWrapper> backPtr;
    backPtr = std::make_shared<BackWrapper>(nh, cfgPath);
    ros::spin();
    return 0;
}

