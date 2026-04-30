/*
 * @Author: ylh 
 * @Date: 2024-04-18 23:39:01 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 22:57:49
 */

#include "fusion_slam/modules/sensors/imu/propagate.h"

Propagate::Propagate(const std::string &configPath){
    lastAcc = Eigen::Vector3d::Zero();
    lastAngvel = Eigen::Vector3d::Zero();
    lastPcdEndTime = 0.0;
    imuAccScale = 1.0;
    rtkType = 0;

    YAML::Node configNode;
    configNode = YAML::LoadFile(configPath);
    for(size_t i = 0; i < 3; ++i){
        rtkTransNoise[i] = configNode["propagate"]["rtk_trans_noise"][i].as<double>();
        rtkRpyNoise[i] = configNode["propagate"]["rtk_rpy_noise"][i].as<double>();
    }
    rtkType = configNode["propagate"]["rtk_type"].as<int>();
    LOG(INFO) <<"\033[1;32m"<< "propagate: " << "\033[0m" << std::endl;
    std::cout << "      rtk_type                :   " << rtkType << std::endl;
    std::cout << "      rtk_trans_noise         : [ " << rtkTransNoise.transpose() << " ]" << std::endl;
    std::cout << "      rtk_rpy_noise           : [ " << rtkRpyNoise.transpose() << " ]" << std::endl;

}

Propagate::~Propagate(){}

int Propagate::run(MeasureGroupAdd &msg, IESKF::Ptr &ieskfPtr){
    auto t0 = GetCurrentTime::nowUs();
    msg.mapDatas.emplace(lastImu.timeStamp.nsec(), DataUnit{0, lastImu});
    std::vector<u_int64_t> datasTimeList;
    for(const auto& [key, value] : msg.mapDatas){
        datasTimeList.emplace_back(key);
    }
    auto& cloud = *msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr;
    double curPcdStartTime = msg.lidarBeginTime, curPcdEndTime = msg.lidarEndTime;
    auto t1 = GetCurrentTime::nowUs();
    std::sort(cloud.begin(), cloud.end(),
                  [](PointType x, PointType y) { return x.offset_time < y.offset_time;});
    auto t2 = GetCurrentTime::nowUs();
    std::vector<IMUPose6d> imuPoses;
    auto state = ieskfPtr->getX();
    imuPoses.clear();
    imuPoses.emplace_back(IMUPose6d{0.0, lastAcc, lastAngvel, state.velocity, state.position, state.rotation});
    Eigen::Vector3d avrAngvel, avrAcc, imuAcc, imuVel, imuPos;
    Eigen::Quaterniond imuRot;
    for(size_t i = 0; i < datasTimeList.size()-1; ++i){
        if(msg.mapDatas[datasTimeList[i]].type == 0){
            auto& head = msg.mapDatas[datasTimeList[i]].imu;
            while(msg.mapDatas[datasTimeList[i+1]].type != 0){
                if(msg.mapDatas[datasTimeList[i+1]].type == 2){// 2 rtk观测
                    Rot tmpRot;
                    Pos tmpPos;
                    tmpRot.timeStamp.fromNsec(datasTimeList[i+1]);
                    tmpRot.rotation = msg.mapDatas[datasTimeList[i+1]].rtkPose.rotation;
                    tmpPos.timeStamp.fromNsec(datasTimeList[i+1]);
                    tmpPos.position = msg.mapDatas[datasTimeList[i+1]].rtkPose.position;
                    if(rtkType == 0){
                        ieskfPtr->positionObserve(tmpPos, rtkTransNoise);
                        ieskfPtr->rotationObserve(tmpRot, rtkRpyNoise);
                    }else if(rtkType == 1){
                        ieskfPtr->positionObserve(tmpPos, rtkTransNoise);
                    }else if(rtkType == 2){
                        ieskfPtr->rotationObserve(tmpRot, rtkRpyNoise);
                    }
                }
                // other observe, eg: velocity
                ++i;
            }

            auto& tail = msg.mapDatas[datasTimeList[i+1]].imu;
            if(tail.timeStamp.nsec() < lastPcdEndTime) continue;
            avrAngvel = 0.5*(head.gyroscope + tail.gyroscope);
            avrAcc = 0.5 * (head.acceleration + tail.acceleration);
            ImuType tmpImu;
            tmpImu.acceleration = avrAcc;
            tmpImu.gyroscope = avrAngvel;
            if(head.timeStamp.sec() < lastPcdEndTime){ 
                tmpImu.timeStamp.fromNsec(tail.timeStamp.nsec());
            }else{
                tmpImu.timeStamp.fromNsec(head.timeStamp.nsec());
            }
            ieskfPtr->predict(tmpImu);
            state = ieskfPtr->getX();
            lastAngvel = avrAngvel - state.bg;
            lastAcc = (avrAcc - state.ba)*imuAccScale;
            for(int i = 0; i < 3; ++i){
                lastAcc[i] += state.gravity[i];
            }
            imuVel = state.velocity;
            imuPos = state.position;
            imuRot = state.rotation;
            auto offetTime = tmpImu.timeStamp.sec() - curPcdStartTime*1e-9;
            imuPoses.emplace_back(IMUPose6d{offetTime, lastAcc, lastAngvel, imuVel, imuPos, imuRot});
            lastImu = tail;
        }
    }
    msg.mapDatas[msg.lidarBeginTime].cloud.timeStamp.fromNsec(msg.lidarEndTime);
    auto t3 = GetCurrentTime::nowUs();
    if(cloud.points.size() < 2) return 0;
    auto&& it_point = cloud.points.end()-1;
    for(auto it = imuPoses.end()-1; it != imuPoses.begin(); --it){
        auto head = it-1;
        auto tail = it;
        for(; it_point->offset_time / 1e9 > head->time; --it_point){
            double dt = it_point->offset_time*1e-9 - head->time;
            Eigen::Matrix3d R_i(head->rot.toRotationMatrix() * so3Exp(tail->angvel * dt));
            Eigen::Vector3d P_i(it_point->x, it_point->y, it_point->z);
            Eigen::Vector3d T_ei(head->pos + head->vel * dt + 0.5 * tail->acc * dt * dt - state.position);
            Eigen::Vector3d P_e_imu = state.rotation.conjugate() * (R_i * P_i + T_ei);
            it_point->x = P_e_imu[0];
            it_point->y = P_e_imu[1];
            it_point->z = P_e_imu[2];
            if(it_point == cloud.points.begin()) break;
        }
    }
    auto t4 = GetCurrentTime::nowUs();
    return 0;
}