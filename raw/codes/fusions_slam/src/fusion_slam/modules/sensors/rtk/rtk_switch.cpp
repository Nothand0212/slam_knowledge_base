/*
 * @Author: ylh 
 * @Date: 2024-05-01 12:00:11 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-01 12:50:32
 */
#include <memory>
#include "fusion_slam/modules/sensors/rtk/rtk_switch.h"

RtkSwitch::RtkSwitch(){
        a = 6378137.0000000000;
        b = 6356752.3142451793;
        e2 = (a*a-b*b)/(a*a);
        f = (a-b)/(a);
    }
RtkSwitch::~RtkSwitch(){}
int RtkSwitch::LLA2Enu(const Eigen::Vector3d& lla, const Eigen::Vector3d& ref_lla, Eigen::Vector3d& enu){
    Eigen::Vector3d d_lla = lla - ref_lla;
    double d_lat = d_lla[0];
    double d_lon = d_lla[1];
    double d_alt = d_lla[2];
    double lat = ref_lla[0];
    double lon = ref_lla[1];
    double alt = ref_lla[2];
    double N = a/sqrt(1-e2*sin(lat)*sin(lat));
    double M = a*(1-e2)/pow(1-e2*sin(lat)*sin(lat),1.5);
    enu[0] = (N+alt)*cos(lat)*d_lon;
    enu[1] = (M+alt)*d_lat;
    enu[2] = d_alt;
    return 0;
}

int RtkSwitch::LLA2Ecef(const Eigen::Vector3d& lla, Eigen::Vector3d& ecef){
    double lat = lla[0];
    double lon = lla[1];
    double alt = lla[2];
    double N = a/sqrt(1-e2*sin(lat)*sin(lat));
    ecef[0] = (N+alt)*cos(lat)*cos(lon);
    ecef[1] = (N+alt)*cos(lat)*sin(lon);
    ecef[2] = (N*(1-e2)+alt)*sin(lat);
    return 0;
}

int RtkSwitch::ecef2Enu(const Eigen::Vector3d& ecef, const Eigen::Vector3d& refLla, const Eigen::Vector3d& refEcef, Eigen::Vector3d& enu){
    Eigen::Vector3d dEcef = ecef - refEcef;
    Eigen::Matrix3d R;
    double lat = refLla[0];
    double lon = refLla[1];
    double alt = refLla[2];
    R << -sin(lon), cos(lon), 0,
            -cos(lon)*sin(lat), -sin(lon)*sin(lat), cos(lat),
            cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat);
    enu = R*dEcef;
    return 0;
}

int RtkSwitch::LLA2Enu2(const Eigen::Vector3d& lla, const Eigen::Vector3d& refLla, Eigen::Vector3d& enu){
    Eigen::Vector3d ecef = Eigen::Vector3d::Zero(), refEcef = Eigen::Vector3d::Zero();
    LLA2Ecef(refLla, refEcef);
    LLA2Ecef(lla, ecef);
    ecef2Enu(ecef, refLla, refEcef, enu);
    return 0;
}