/*
 * @Author: ylh 
 * @Date: 2024-04-09 23:26:09 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-04 23:53:57
 */

#include "fusion_slam/modules/back/lio_gps_opt.h"

LioGpsOpt::LioGpsOpt(){
    hasNewLioPose = false; 
    hasNewGpsPose = false;
    hasNewGpsOdom = false; // 相应lio时刻的gps全局位姿
    WGPS_T_WLIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&LioGpsOpt::optimize, this);
}

LioGpsOpt::~LioGpsOpt(){
    threadOpt.detach();
}

Eigen::Vector3d LioGpsOpt::interpPosition(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    double alpha)
{
    return (1.0 - alpha) * p0 + alpha * p1;
}

Eigen::Quaterniond LioGpsOpt::interpQuaternion(
    const Eigen::Quaterniond& q0,
    const Eigen::Quaterniond& q1,
    double alpha)
{
    return q0.slerp(alpha, q1);
}

bool LioGpsOpt::syncMeasurePose(MeasurePose &msg){//高频lio时，低频gps可能会有问题
    if(lioPoseBuf.empty() || gpsPoseBuf.size() < 2){
        return false;
    }
    mLioPose.lock();
    msg.ts = lioPoseBuf.front().timeStamp.sec();
    msg.lio = lioPoseBuf.front();
    alignPose(msg.ts, msg.gps);
    lioPoseBuf.pop_front();
    mLioPose.unlock();
    return true;
}

void LioGpsOpt::optimize(){
    while (true)
    {
        if(hasNewGpsOdom)
        {
            hasNewGpsOdom = false;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mLioPose.lock();
            int length = lioPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, Pose>::iterator iter;
            iter = globalLioPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)// 将lio全局系统下位姿添加到优化问题中
            {
                t_array[i][0] = iter->second.position[0];
                t_array[i][1] = iter->second.position[1];
                t_array[i][2] = iter->second.position[2];
                q_array[i][0] = iter->second.rotation.w();
                q_array[i][1] = iter->second.rotation.x();
                q_array[i][2] = iter->second.rotation.y();
                q_array[i][3] = iter->second.rotation.z();
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, Pose>::iterator iterLIO, iterLIONext, iterGPS;
            int i = 0;
            for (iterLIO = lioPoseMap.begin(); iterLIO != lioPoseMap.end(); iterLIO++, i++)
            {
                //lio factor
                iterLIONext = iterLIO;
                iterLIONext++;
                if(iterLIONext != lioPoseMap.end())// 将雷达坐标系下lio位姿增量添加到优化问题中
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = iterLIO->second.rotation.toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = iterLIO->second.position;
                    wTj.block<3, 3>(0, 0) = iterLIONext->second.rotation.toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = iterLIONext->second.position;

                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* lio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(lio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                }
                //gps factor
                double t = iterLIO->first;
                iterGPS = gpsPoseMap.find(t);//找到lio时刻对应的gps位置信息（注意优化器输入前已做时间戳对齐）
                if (iterGPS != gpsPoseMap.end())// 找到全局gps位置信息，添加到优化问题中
                {
                    double x = iterGPS->second.position.x(), y = iterGPS->second.position.y(), z = iterGPS->second.position.z();
                    ceres::CostFunction* gps_function = TError::Create(x, y, z, 1.0);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);
                }

            }
            ceres::Solve(options, &problem, &summary); // 优化
            //std::cout << summary.BriefReport() << "\n";

            // update global pose
            iter = globalLioPoseMap.begin();
            for (int i = 0; i < length; i++, iter++) // 更新Lio全局位姿
            {
            	std::vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
                Pose globalLio;
                globalLio.timeStamp.fromSec(iter->first);
                globalLio.position = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
                globalLio.rotation = Eigen::Quaterniond(globalPose[3], globalPose[4], globalPose[5], globalPose[6]);
            	iter->second = globalLio;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WLIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
                    WLIO_T_body.block<3, 3>(0, 0) = lioPoseMap[t].rotation.toRotationMatrix();
                    WLIO_T_body.block<3, 1>(0, 3) = lioPoseMap[t].position;
                    WGPS_T_body.block<3, 3>(0, 0) = globalLio.rotation.toRotationMatrix();
                    WGPS_T_body.block<3, 1>(0, 3) = globalLio.position;
            	    WGPS_T_WLIO = WGPS_T_body * WLIO_T_body.inverse();// WGPS_T_WLIO是时间戳t时，LIO系到全局系的变换矩阵
                    // LOG(INFO) << "WGPS_T_WLIO: \n" << WGPS_T_WLIO;
            	}
            }
            mLioPose.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    } 
}

bool LioGpsOpt::alignPose(const double &ts, Pose &out){
    // todo : 高频lio时，低频gps可能会有问题.因此只插值在lio时刻0.01s左右的gps因子
    Pose left, right;
    mGpsPose.lock();
    if (gpsPoseBuf.size() < 2) return false;
    // 边界检查
    if (ts <= gpsPoseBuf.front().timeStamp.sec()) {
        left  = gpsPoseBuf.front();
        right = gpsPoseBuf.back();
    }
    else if (ts >= gpsPoseBuf.back().timeStamp.sec()) {
        left  = gpsPoseBuf[gpsPoseBuf.size() - 2];
        right = gpsPoseBuf.back();
    }else{
        // 二分查找第一个 timeStamp >= ts 的元素
        auto iter = std::lower_bound(
        gpsPoseBuf.begin(), gpsPoseBuf.end(), ts,
        [](const Pose& p, double value) {
            return p.timeStamp.sec() < value;
        });
        right = *iter;
        left  = *(iter - 1);
    }
    while(gpsPoseBuf.front().timeStamp.sec() < ts - 1.0){
        gpsPoseBuf.pop_front();
    }
    mGpsPose.unlock();
    // 插值
    if (abs(left.timeStamp.sec() -ts) > 0.01 && abs(right.timeStamp.sec() -ts) > 0.01) return false;
    double alpha = (ts - left.timeStamp.sec()) / (right.timeStamp.sec() - left.timeStamp.sec());
    out.position = interpPosition(left.position, right.position, alpha);
    out.rotation = interpQuaternion(left.rotation, right.rotation, alpha);
    out.timeStamp.fromSec(ts);
    return true;
}

void LioGpsOpt::inLio(const Pose &in){
    mLioPose.lock();
    lioPoseMap[in.timeStamp.sec()] = in;
    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WLIO.block<3, 3>(0, 0) * in.rotation;
    Eigen::Vector3d globalP = WGPS_T_WLIO.block<3, 3>(0, 0) * in.position + WGPS_T_WLIO.block<3, 1>(0, 3);
    Pose globalLio;
    globalLio.timeStamp.fromSec(in.timeStamp.sec());
    globalLio.rotation = globalQ;
    globalLio.position = globalP;
    globalLioPoseMap[in.timeStamp.sec()] = globalLio;// 全局位姿，输入为lidar位姿，根据WGPS_T_WLIO矩阵转为全局
    lastPose = globalLio;
    Pose gpsOdom;
    if(alignPose(in.timeStamp.sec(), gpsOdom)){
        gpsPoseMap[in.timeStamp.sec()] = gpsOdom;
        hasNewGpsOdom = true;
    }
    mLioPose.unlock();
}

void LioGpsOpt::inGpsOdom(const Pose &in){
    mGpsPose.lock();
    gpsPoseBuf.push_back(in);
    mGpsPose.unlock();
}

void LioGpsOpt::getGlobalPose(Pose &out){
    out = lastPose;
}

