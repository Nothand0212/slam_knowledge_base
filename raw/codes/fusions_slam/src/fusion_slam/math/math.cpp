/*
 * @Author: ylh 
 * @Date: 2024-05-01 12:31:29 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-01 12:32:19
 */
#include "fusion_slam/math/math.h"

void rpy2q(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q){
    double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
}

void q2rpy(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy){
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    double sinp = std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
    double cosp = std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
    double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    rpy << roll, pitch, yaw;
}

void q2R(const Eigen::Quaterniond& q_, Eigen::Matrix3d& R)
{
    Eigen::Quaterniond q = q_.normalized();
    R(0,0) = 1-2*(q.y()*q.y() + q.z()*q.z());
    R(0,1) = 2*(q.x()*q.y() - q.w()*q.z());
    R(0,2) = 2*(q.w()*q.y() + q.x()*q.z());
    R(1,0) = 2*(q.x()*q.y() + q.w()*q.z());
    R(1,1) = 1 - 2*(q.x()*q.x() + q.z()*q.z());
    R(1,2) = 2*(q.y()*q.z() - q.w()*q.x());
    R(2,0) = 2*(q.x()*q.z() - q.w()*q.y());
    R(2,1) = 2*(q.w()*q.x() + q.y()*q.z());
    R(2,2) = 1- 2*(q.x()*q.x() + q.y()*q.y());
}

void R2q(const Eigen::Matrix3d& R, Eigen::Quaterniond& q)
{
    q.w() = std::sqrt(1 + R(0,0) + R(1,1) + R(2,2))/2;
    q.x() = (R(2,1) - R(1,2))/(4*q.w());
    q.y() = (R(0,2) - R(2,0))/(4*q.w());
    q.z() = (R(1,0) - R(0,1))/(4*q.w());
}

void q2AxisAngle(const Eigen::Quaterniond& q, Eigen::AngleAxisd& aa)
{
    double angle = 2*acos(q.w());
    Eigen::Vector3d axis;
    double sa = sin(angle/2);
    if(sa < 1e-6){
        std::cout << " q.w() too little ! " << std::endl;
        aa = Eigen::AngleAxisd(0,Eigen::Vector3d(0,0,1));
        return;
    }
    axis[0] = q.x()/sa;
    axis[1] = q.y()/sa;
    axis[2] = q.z()/sa;
    aa = Eigen::AngleAxisd(angle, axis);
}

void axisAngle2q(const Eigen::AngleAxisd& aa, Eigen::Quaterniond& q)
{
    Eigen::Vector3d axis = aa.axis();
    axis = axis.normalized();
    double angle = aa.angle();
    double sa = sin(angle/2);
    double ca = cos(angle/2);
    q.x() = sa*axis[0];
    q.y() = sa*axis[1];
    q.z() = sa*axis[2];
    q.w() = ca;
}

void rpy2R(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R){
    Eigen::Quaterniond q;
    rpy2q(rpy, q);
    q2R(q, R);
}

void R2rpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy){
    Eigen::Quaterniond q;
    R2q(R, q);
    q2rpy(q, rpy);
}

//将角度限制在[-PI,PI]
double normalizeRadian(double rad) {
    rad = std::fmod(rad + PI, 2.0 * PI);
    if (rad < 0)
        rad += 2.0 * PI;
    return rad - PI;
}

// boreas
void roll2Rot(const double& roll, Eigen::Matrix3d& R){
   R << 1, 0, 0, 0, cos(roll), sin(roll), 0, -sin(roll), cos(roll);

}
void pitch2Rot(const double& pitch, Eigen::Matrix3d& R){
   R << cos(pitch), 0, -sin(pitch), 0, 1, 0, sin(pitch), 0, cos(pitch);
}
void yaw2Rot(const double& yaw, Eigen::Matrix3d& R){
   R << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1;
}
void rpy2Rot(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R){
   Eigen::Matrix3d R1, R2, R3;
   roll2Rot(rpy[0], R1);
   pitch2Rot(rpy[1], R2);
   yaw2Rot(rpy[2], R3);
   R = R1 * R2 * R3;
}

void rot2rpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy){
    int i = 2, j = 1, k = 0;
    double cy = std::sqrt(R(i, i)*R(i, i) + R(j, i)*R(j, i));
    if(cy > 1e-14){
        rpy[0] = std::atan2(R(j, i), R(i, i));
        rpy[1] = std::atan2(-R(k, i), cy);
        rpy[2] = std::atan2(R(k, j), R(k, k));
    }else{
        rpy[0] = 0;
        rpy[1] = std::atan2(-R(k, i), cy);
        rpy[2] = std::atan2(-R(j, k), R(j, j));
    }
}
// boreas