/*
 * @Author: ylh 
 * @Date: 2024-09-13 15:27:38 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-13 21:29:03
 */

#ifndef LOCATION_H
#define LOCATION_H
#include "fusion_slam/type/base_type.h"
#include "fusion_slam/type/pose.h"
#include "fusion_slam/type/pointcloud.h"
#include "fusion_slam/type/velocity.h"
#include "fusion_slam/type/measure_group.h"
#include "fusion_slam/modules/ieskf/ieskf.h"
#include "fusion_slam/modules/sensors/imu/imu_static_init.h"
#include "fusion_slam/modules/map/map_engine.h"
#include "fusion_slam/modules/sensors/imu/propagate.h"

class Location{
    private:
        std::deque<ImuType> imus;
        std::deque<PointCloud> clouds;
        std::deque<Pose> rtkPoses;
        std::deque<Velocity> vels;
        bool inited;
        VoxelFilter pcdVoxelFilter;
        PCLPointCloudPtr pcdPtrFilterResult;
        bool initUseRtk;
        double imuInitDuration;
        double imuAccScale;
        IESKF::Ptr locIeskfPtr;
        ImuStaticInit::Ptr locImuStaticInitPtr;
        MapEngine::Ptr locMapEnginePtr;
        Propagate::Ptr locPropagatePtr;
        
    public:
        using Ptr = std::shared_ptr<Location>;
        Location(const std::string& configPath);
        ~Location();
        
        int addImu(const ImuType& imu);
        int addLidar(const PointCloud& cloud);
        int addRtk(const Pose& pose);
        int addVelocity(const Velocity& vel);

        int init(MeasureGroupAdd &msg);
        bool syncMeasureGroupAdd(MeasureGroupAdd &msg);
        bool run();
        const PCLPointCloud& readCurrentLocalMap();
        const PCLPointCloud& readCurrentPointCloud();
        StateX readState();
};
#endif