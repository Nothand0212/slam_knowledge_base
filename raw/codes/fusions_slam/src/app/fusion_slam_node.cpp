/*
 * @Author: ylh 
 * @Date: 2024-04-07 21:59:50 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-04-08 23:09:07
 */

#include "wrapper/ros/fusion_slam_ros_wrapper.h"
int main(int argc, char *argv[])
{
    if(argc<2){
        LOG(ERROR)<<"Usage: ./fusion_slam_node [config.yaml]";
        return -1;
    }
    std::string cfgPath = argv[1];
    ros::init(argc,argv,"fusion_slam_node");
    ros::NodeHandle nh;
    std::shared_ptr<FusionSlamFrontWrapper> frontPtr;
    frontPtr = std::make_shared<FusionSlamFrontWrapper>(nh, cfgPath);
    return 0;
}