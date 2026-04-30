/*
 * @Author: ylh 
 * @Date: 2024-04-07 21:59:50 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-13 22:28:43
 */

#include "wrapper/ros/location_ros_wrapper.h"
int main(int argc, char *argv[])
{
    if(argc<2){
        LOG(ERROR)<<"Usage: ./location_node [config.yaml]";
        return -1;
    }
    std::string cfgPath = argv[1];
    ros::init(argc,argv,"location_node");
    ros::NodeHandle nh;
    std::shared_ptr<LocationWrapper> locationPtr;
    locationPtr = std::make_shared<LocationWrapper>(nh, cfgPath);
    return 0;
}