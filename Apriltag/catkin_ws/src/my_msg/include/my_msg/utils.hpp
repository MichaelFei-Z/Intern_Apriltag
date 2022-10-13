/**
 * @file utils.cpp
 * @author Michael
 * @brief 旋转矩阵与欧拉角相互转换
 * @version 0.1
 * @date 2022-09-19
 * @ref https://zhuanlan.zhihu.com/p/144032401
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace utils
{
    using namespace std;

    /**
     * @brief 欧拉角 -> 旋转矩阵
     * 
     * @param theta : 欧拉角（弧度单位，外旋 x -> y -> z）
     * @return Eigen::Matrix3d ：旋转矩阵（3X3）
     */
    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
    {
        Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
        R_x <<
                1,              0,               0,
                0,  cos(theta[0]),  -sin(theta[0]),
                0,  sin(theta[0]),   cos(theta[0]);

        Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
        R_y <<
                cos(theta[1]),   0, sin(theta[1]),
                0,   1,             0,
                -sin(theta[1]),  0, cos(theta[1]);

        Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
        R_z <<
                cos(theta[2]), -sin(theta[2]), 0,
                sin(theta[2]),  cos(theta[2]), 0,
                0,              0,             1;
        Eigen::Matrix3d R = R_z * R_y * R_x;
        return R;
    }

    /**
     * @brief 判断旋转矩阵是否合理
     * 
     * @param R ：旋转矩阵（3X3）
     * @return true 
     * @return false 
     */
    bool isRotationMatirx(Eigen::Matrix3d R)
    {
        double err=1e-6;
        Eigen::Matrix3d shouldIdenity;
        shouldIdenity=R*R.transpose();
        Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
        return (shouldIdenity - I).norm() < err;
    }

    /**
     * @brief 旋转矩阵 -> 欧拉角
     * 
     * @param R ：旋转矩阵（3X3）
     * @return Eigen::Vector3d ：欧拉角（弧度，外旋 x -> y -> z）
     */
    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
    {
        assert(isRotationMatirx(R));
        double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        double x, y, z;
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        return {x, y, z};
    }
}

#endif /* UTILS_HPP_ */