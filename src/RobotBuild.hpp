#ifndef ROBOTBUILD_HPP
#define ROBOTBUILD_HPP

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <unordered_map>
#include "BodyMotion.hpp"

using namespace std;
using namespace Eigen;

class RobotDH
{
private:
    /* data */
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotDH(string robot);
    RobotDH();
    ~RobotDH();

    string robotName;  // 机器人名字
    int DoF;           // 机器人自由度
    // 机械臂DH参数
    VectorXf theta;  // DH参数之关节角
    VectorXf d;      // DH参数之连杆偏移
    VectorXf a;      // DH参数之连杆长度
    VectorXf alpha;  // DH参数之连杆扭转角

    Matrix4f FKSlover(float *pTheta);
};


#endif