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
    RobotDH(string robot = "PUMA560");
    ~RobotDH();

    string robotName;  // 机器人名字
    int DoF;           // 机器人自由度
    // 动态大小的数组是在堆区开辟，故可以再用个等号来声明大小
    VectorXf theta;  // DH参数之关节角
    VectorXf d;      // DH参数之连杆偏移
    VectorXf a;      // DH参数之连杆长度
    VectorXf alpha;  // DH参数之连杆扭转角

    Matrix4f FKSlover(float *pTheta);
};


#endif