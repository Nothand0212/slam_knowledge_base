#ifndef SO3_H
#define SO3_H
#include <Eigen/Dense>

static inline Eigen::Matrix3d  skewSymmetric(const Eigen::Vector3d& so3){ 
    Eigen::Matrix3d so3SkewSym;
    so3SkewSym.setZero();
    so3SkewSym(0,1) = -1*so3(2);
    so3SkewSym(1,0) = so3(2);
    so3SkewSym(0,2) = so3(1);
    so3SkewSym(2,0) = -1*so3(1);
    so3SkewSym(1,2) = -1*so3(0);
    so3SkewSym(2,1) = so3(0);
    return so3SkewSym;
}

static Eigen::Matrix3d so3Exp(const Eigen::Vector3d& so3){
    Eigen::Matrix3d SO3;
    double so3Norm = so3.norm();
    if (so3Norm <= 1e-7)
    {
        SO3.setIdentity();
        return SO3;
        // return Eigen::Matrix3d::Identity() + skewSymmetric(so3);// 精度提升？
    }
    Eigen::Matrix3d so3SkewSym = skewSymmetric(so3);
    SO3 = Eigen::Matrix3d::Identity()+(so3SkewSym/so3Norm)*sin(so3Norm)+(so3SkewSym*so3SkewSym/(so3Norm*so3Norm))*(1-cos(so3Norm));
    return SO3;
}

static Eigen::Vector3d SO3Log(const Eigen::Matrix3d& SO3){
    double theta = (SO3.trace()>3-1e6)?0:acos((SO3.trace()-1)/2);
    Eigen::Vector3d so3(SO3(2,1)-SO3(1,2),SO3(0,2)-SO3(2,0),SO3(1,0)-SO3(0,1)); 
    return fabs(theta)<0.001?(0.5*so3):(0.5*theta/sin(theta)*so3);
}

static Eigen::Matrix3d  J_right(const Eigen::Vector3d& v){
    Eigen::Matrix3d res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squaredNorm);
    if(norm <1e-11){
        res = Eigen::Matrix3d::Identity();
    }
    else{
        res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squaredNorm * skewSymmetric(v) + (1 - std::sin(norm) / norm) / squaredNorm * skewSymmetric(v) * skewSymmetric(v);
    }
    return res;
}

#endif