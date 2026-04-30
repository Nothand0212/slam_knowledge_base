/*
 * @Author: ylh 
 * @Date: 2024-04-07 22:37:24 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 22:59:55
 */
#include "wrapper/ros/fusion_slam_ros_wrapper.h"

FusionSlamFrontWrapper::FusionSlamFrontWrapper(ros::NodeHandle &nh, const std::string& cfgPath){
    std::string configPath;
    YAML::Node configNode;
    configPath = cfgPath;
    std::string imuTopic, lidarTopic, rtkTopic;
    rtkTopic = "/novatel_data/inspvax";
    imuTopic = "/imu/data";
    lidarTopic = "/velodyne_points";
    savePose = "";
    lidarLineNum = 32;
    pointGapNum = -1;
    configNode = YAML::LoadFile(configPath);
    savePose = configNode["wrapper"]["save_pose"].as<bool>();
    lidarLineNum = configNode["wrapper"]["lidar_line_num"].as<int>();
    pointGapNum = configNode["wrapper"]["point_gap_num"].as<int>();
    rtkTopic = configNode["wrapper"]["rtk_topic"].as<std::string>();
    imuTopic = configNode["wrapper"]["imu_topic"].as<std::string>();
    lidarTopic = configNode["wrapper"]["lidar_topic"].as<std::string>();
    lidarType = configNode["wrapper"]["lidar_type"].as<int>();
    rtkDateType = configNode["wrapper"]["rtk_date_type"].as<int>();
    useRtk = configNode["wrapper"]["use_rtk"].as<bool>();
    for (size_t i = 0; i < 16; ++i) {
        T_imu_lidar(i / 4, i % 4) = configNode["wrapper"]["T_imu_lidar"][i].as<double>();
        T_imu_ant(i / 4, i % 4) = configNode["wrapper"]["T_imu_ant"][i].as<double>();
    }
    for(size_t i = 0; i < 4; ++i){
        lidarXYBox[i] = configNode["wrapper"]["lidar_xy_box"][i].as<double>();
    }
    
    frontPtr = std::make_shared<Front>(configPath);
    rtkSwitchPtr = std::make_shared<RtkSwitch>();
    velodynelidarProcessPtr = std::make_shared<VelodyneProcess>();
    ousterlidarProcessPtr = std::make_shared<OusterProcess>();
    velodynelidarProcessPtr->init(lidarLineNum, pointGapNum);
    ousterlidarProcessPtr->init(lidarLineNum, pointGapNum);

    cloudSubscriber = nh.subscribe(lidarTopic, 100000, &FusionSlamFrontWrapper::lidarCloudMsgCallBack, this);
    imuSubscriber = nh.subscribe(imuTopic, 100000, &FusionSlamFrontWrapper::imuMsgCallBack, this);
    if(rtkDateType == 1){
        rtkInspvaxSubscriber = nh.subscribe(rtkTopic, 100000, &FusionSlamFrontWrapper::rtkInspvaxMsgCallBack, this);
    }else if(rtkDateType == 2){
        rtkNavsatSubscriber = nh.subscribe(rtkTopic, 100000, &FusionSlamFrontWrapper::rtkNavsatMsgCallBack, this);
    }else if(rtkDateType == 3){
        rtkOdomSubscriber = nh.subscribe(rtkTopic, 100000, &FusionSlamFrontWrapper::rtkOdomMsgCallBack, this);
    }
    
    currCloudPub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
    pathPub = nh.advertise<nav_msgs::Path>("path", 100);
    rtkPathPub = nh.advertise<nav_msgs::Path>("rtk_path", 100);
    localMapPub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);
    lioOdomPub = nh.advertise<nav_msgs::Odometry>("lio_odom", 100);
    gpsInitSuccess = false;

    LOG(INFO) <<"\033[1;32m"<< "wrapper: " << "\033[0m" << std::endl;
    std::cout << "      lidar_line_num  : " << lidarLineNum << std::endl;
    std::cout << "      point_gap_num   : " << pointGapNum << std::endl;
    std::cout << "      lidar_topic     : " << lidarTopic << std::endl;
    std::cout << "      imu_topic       : " << imuTopic << std::endl;
    std::cout << "      rtk_topic       : " << rtkTopic << std::endl;
    std::cout << "      lidar_xy_box    : " << "[ " << lidarXYBox.transpose() << " ]" << std::endl;
    std::cout << "      use_rtk         : " << useRtk << std::endl; 
    std::cout << "      lidar_type      : " << lidarType << std::endl; 
    std::cout << "      rtk_date_type   : " << rtkDateType << std::endl;
    std::cout << "      T_imu_lidar     : " << std::endl;
    std::cout << "[ "<< std::endl;
    std::cout << T_imu_lidar << std::endl;
    std::cout << " ]" <<std::endl;
    std::cout << "      T_imu_ant       : " << std::endl;
    std::cout << "[ "<< std::endl;
    std::cout << T_imu_ant << std::endl;
    std::cout << " ]" <<std::endl; 
    run();
}

int FusionSlamFrontWrapper::init(const std::string& path){
    cfgPath = path;
    return 0;
}

FusionSlamFrontWrapper::~FusionSlamFrontWrapper(){}

time_t FusionSlamFrontWrapper::gps2unix(const int& gpsWeek, const double& gpsTow) {
    return GPS_EPOCH_UNIX + gpsWeek * 7 * 24 * 3600 + static_cast<time_t>(gpsTow) - 18;
}

void FusionSlamFrontWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr& msg){
    PointCloud oriCloud, cloud;
    if(lidarType == 1){
        velodynelidarProcessPtr->process(msg, oriCloud);
    }
    if(lidarType == 2){
        ousterlidarProcessPtr->process(msg, oriCloud);
    }
    cloud.timeStamp = oriCloud.timeStamp;
    for(auto& p : oriCloud.cloudPtr->points){
        if(p.x > lidarXYBox[0] && p.x < lidarXYBox[1] && p.y > lidarXYBox[2] && p.y < lidarXYBox[3]) continue;
        if(isnan(p.x) || isnan(p.y) || isnan(p.z)) continue;
        cloud.cloudPtr->points.emplace_back(p);
    }
    if(cloud.cloudPtr->points.empty()) return;
    pcl::transformPointCloud(*cloud.cloudPtr, *cloud.cloudPtr, T_imu_lidar.cast<float>());
    if(useRtk){
        pcl::transformPointCloud(*cloud.cloudPtr, *cloud.cloudPtr, T_imu_ant.inverse().cast<float>());
    }
    frontPtr->addLidar(cloud);
}

void FusionSlamFrontWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg){
    ImuType imu;
    imuProcessPtr->process(msg, imu);
    frontPtr->addImu(imu);
}

void FusionSlamFrontWrapper::rtkInspvaxMsgCallBack(const novatel_msgs::INSPVAX::ConstPtr& msg){
    if(!useRtk) return;
    static bool firstGps = true;
    static Eigen::Vector3d refLla, refRpy;
    static int gpsWeek;
    static double gpsTow;
    static time_t unixTime;
    static Pose enuPose;
    if (firstGps) {
        refLla << msg->latitude*PI/180, msg->longitude*PI/180, msg->altitude;
        refRpy << msg->pitch*PI/180, msg->roll*PI/180,  normalizeRadian(PI/2 - msg->azimuth*PI/180);
        std::cout << " first rtk rot " << refRpy.transpose()*PI/180 << std::endl;
        firstGps = false;
        return;
    }
    gpsWeek = msg->header.gps_week;
    gpsTow = msg->header.gps_week_seconds*1e-3;
    unixTime = gps2unix(gpsWeek, gpsTow);
    static Eigen::Vector3d lla, rpy, vel, enu2;
    Eigen::Quaterniond q;
    lla << msg->latitude*PI/180, msg->longitude*PI/180, msg->altitude;
    rpy << msg->pitch*PI/180, msg->roll*PI/180,  normalizeRadian(PI/2 - msg->azimuth*PI/180); // NED --> ENU
    vel << msg->east_velocity, msg->north_velocity, msg->up_velocity;
    rtkSwitchPtr->LLA2Enu2(lla, refLla, enu2);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    rpy2R(rpy,R);// 需要根据数据集对应的转换方式进行修改，此处用法不一定正确
    Eigen::Matrix4d T_ENU_rtkPose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_map_rtkPose = Eigen::Matrix4d::Identity();
    T_ENU_rtkPose.block<3, 3>(0, 0) = R;
    T_ENU_rtkPose.block<3, 1>(0, 3) = enu2;
    T_map_rtkPose = T_ENU_rtkPose*T_imu_ant.inverse();
    R = T_map_rtkPose.block<3, 3>(0, 0);
    enu2 = T_map_rtkPose.block<3, 1>(0, 3);
    R2q(R, q);
    enuPose.timeStamp.fromSec(double(unixTime));
    enuPose.position = enu2;
    enuPose.rotation = q;
    frontPtr->addRtk(enuPose);
    // test
    static nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp.fromSec(double(unixTime));
    geometry_msgs::PoseStamped psd;
    psd.pose.position.x = enu2[0];
    psd.pose.position.y = enu2[1];
    psd.pose.position.z = enu2[2];
    path.poses.emplace_back(psd);
    rtkPathPub.publish(path);
    if(savePose){
        static std::string rtkPosePath = std::string(ROOT_DIR) + "/data/rtk_pose.txt";
        static std::ofstream ofs(rtkPosePath, std::ios::trunc);
        ofs << std::fixed << std::setprecision(9)  << enuPose.timeStamp.sec() << " " << enuPose.position.transpose() << " " <<rpy.transpose() << std::endl;
        static std::string rtkDataPath = std::string(ROOT_DIR) +"/data/rtk_data.txt";
        static std::ofstream ofs_(rtkDataPath, std::ios::trunc);

        double yaw = 90 - msg->azimuth;
        if(msg->azimuth > 90 && msg->azimuth < 180) yaw = 450 - msg->azimuth;
        ofs_ << std::fixed << std::setprecision(9)  << "GNSS " << enuPose.timeStamp.sec() << " " <<  msg->latitude << " " << msg->longitude << " " << msg->altitude << " " << yaw << " 1"<< std::endl;
    }
}

void FusionSlamFrontWrapper::rtkNavsatMsgCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if(!useRtk) return;
    static bool firstGps = true;
    static Eigen::Vector3d refLla, refRpy;
    static Pose enuPose;
    if (firstGps) {
        refLla << msg->latitude*PI/180, msg->longitude*PI/180, msg->altitude;
        firstGps = false;
        return;
    }
    static Eigen::Vector3d lla, rpy, vel, enu2;
    lla << msg->latitude*PI/180, msg->longitude*PI/180, msg->altitude;
    rtkSwitchPtr->LLA2Enu2(lla, refLla, enu2);
    Eigen::Quaterniond q;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d T_ENU_rtkPose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_map_rtkPose = Eigen::Matrix4d::Identity();
    T_ENU_rtkPose.block<3, 3>(0, 0) = R;
    T_ENU_rtkPose.block<3, 1>(0, 3) = enu2;
    T_map_rtkPose = T_ENU_rtkPose*T_imu_ant.inverse();
    R = T_map_rtkPose.block<3, 3>(0, 0);
    enu2 = T_map_rtkPose.block<3, 1>(0, 3);
    R2q(R, q);
    enuPose.timeStamp.fromSec(msg->header.stamp.toSec());
    enuPose.position = enu2;
    enuPose.rotation = q;
    frontPtr->addRtk(enuPose);
    static nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp.fromSec(msg->header.stamp.toSec());
    geometry_msgs::PoseStamped psd;
    psd.pose.position.x = enu2[0];
    psd.pose.position.y = enu2[1];
    psd.pose.position.z = enu2[2];
    path.poses.emplace_back(psd);
    rtkPathPub.publish(path);
    if(savePose){
        static std::string rtkPosePath = std::string(ROOT_DIR) + "/data/rtk_pose.txt";
        static std::ofstream ofs(rtkPosePath, std::ios::trunc);
        ofs << std::fixed << std::setprecision(9)  << enuPose.timeStamp.sec() << " " << enuPose.position.transpose() << " " <<rpy.transpose() << std::endl;
    }
}

void FusionSlamFrontWrapper::rtkOdomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg){
    if(!useRtk) return;
    static Pose enuPose;
    Eigen::Vector3d enuPos, enuVel;
    enuPos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    enuVel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    enuPose.timeStamp.fromSec(msg->header.stamp.toSec());
    enuPose.position = enuPos;
    enuPose.rotation.x() = msg->pose.pose.orientation.x;
    enuPose.rotation.y() = msg->pose.pose.orientation.y;
    enuPose.rotation.z() = msg->pose.pose.orientation.z;
    enuPose.rotation.w() = msg->pose.pose.orientation.w;
    frontPtr->addRtk(enuPose);
    static nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp.fromSec(msg->header.stamp.toSec());
    geometry_msgs::PoseStamped psd;
    psd.pose.position.x = enuPos[0];
    psd.pose.position.y = enuPos[1];
    psd.pose.position.z = enuPos[2];
    path.poses.emplace_back(psd);
    rtkPathPub.publish(path);
    if(savePose){
        static Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
        q2rpy(enuPose.rotation, rpy);
        static std::string rtkPosePath = std::string(ROOT_DIR) + "/data/rtk_pose.txt";
        static std::ofstream ofs(rtkPosePath, std::ios::trunc);
        ofs << std::fixed << std::setprecision(9)  << enuPose.timeStamp.sec() << " " << enuPose.position.transpose() << " " <<rpy.transpose() << std::endl;
    }
}

void FusionSlamFrontWrapper::run(){
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        auto t0 = GetCurrentTime::nowUs();
        if (frontPtr->run()) {
            auto t1 = GetCurrentTime::nowUs();
            publishMsg();
            auto t2 = GetCurrentTime::nowUs();
        }
        rate.sleep();
    }
    frontPtr->mapPtr->saveGlobalMap();
}

void FusionSlamFrontWrapper::publishMsg(){
        static nav_msgs::Path path;
        auto X = frontPtr->readState();
        auto ts = frontPtr->readCurrentTime();
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped psd;
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        path.poses.emplace_back(psd);
        pathPub.publish(path);
        nav_msgs::Odometry lioOdom;
        lioOdom.header.frame_id = "map";
        lioOdom.header.stamp.fromSec(ts);
        lioOdom.pose.pose.position.x = X.position.x();
        lioOdom.pose.pose.position.y = X.position.y();
        lioOdom.pose.pose.position.z = X.position.z();
        lioOdom.pose.pose.orientation.x = X.rotation.x();
        lioOdom.pose.pose.orientation.y = X.rotation.y();
        lioOdom.pose.pose.orientation.z = X.rotation.z();
        lioOdom.pose.pose.orientation.w = X.rotation.w();
        lioOdomPub.publish(lioOdom);
        Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
        q2rpy(X.rotation, rpy);
        if(savePose){
            static std::string curPosePath = std::string(ROOT_DIR) + "/data/cur_pose.txt";
            static std::ofstream ofs(curPosePath, std::ios::trunc);
            ofs << std::fixed << std::setprecision(9)  << ts << " " << X.position.transpose() << " " << rpy.transpose() << std::endl;
            
            // 基于boreas 数据集校验建图效果
            static std::string curPosePathBoreas = std::string(ROOT_DIR) + "/data/cur_pose_boreas.txt";
            static std::ofstream ofsBoreas(curPosePathBoreas, std::ios::trunc);
            static bool firstLine = true;
            if(firstLine){
                ofsBoreas << "GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,roll,pitch,heading,angvel_z,angvel_y,angvel_x,accelz,accely,accelx,latitude,longitude" << std::endl;
                firstLine = false;
            }
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            q2R(X.rotation, R);
            rot2rpy(R, rpy);
            ofsBoreas << std::fixed << std::setprecision(9)  << ts << "," <<X.position.x() << "," << X.position.y() << "," << X.position.z() << 
                ",0,0,0," << rpy(0) << "," << rpy(1) << "," << rpy(2) <<",0,0,0,0,0,0,0,0"<< std::endl;
        }
        PCLPointCloud cloud;
        cloud = frontPtr->readCurrentPointCloud();
        pcl::transformPointCloud(cloud, cloud, quaternionTrans2Mat4d(X.rotation, X.position).cast<float>());
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        msg.header.stamp.fromSec(ts);
        currCloudPub.publish(msg);
        cloud = frontPtr->readCurrentLocalMap();
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        localMapPub.publish(msg);
}
