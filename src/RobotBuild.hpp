#ifndef ROBOTBUILD_HPP
#define ROBOTBUILD_HPP

#include <iostream>
#include <cmath>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include "BodyMotion.hpp"

using namespace std;
using namespace Eigen;

// 基于DH模型的机器人建立类
class RobotDH
{
private:
    
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotDH(string robot);
    RobotDH();
    ~RobotDH();

    void RobotDHInit();  // 根据机械臂名字进行DH参数初始化

    string robotName;  // 机器人名字
    int DoF;           // 机器人自由度 
    VectorXf theta;    // DH参数之关节角
    VectorXf d;        // DH参数之连杆偏移
    VectorXf a;        // DH参数之连杆长度
    VectorXf alpha;    // DH参数之连杆扭转角

};

// 基于旋量的机器人运动学建模
class RobotTwist
{
private:
    /* data */
public:
    RobotTwist(string robot);
    RobotTwist();
    ~RobotTwist();

    void RobotTwistInit();
    Matrix4f FKTwist();

    string robotName;
    int DoF;

    VectorXf theta;   // 旋量角度
    VectorXf omega;   // 角速度和线速度组成的螺旋轴
    VectorXf velocity;

    Matrix4f initSE3;  // 初始末端相对于基座标系的齐次变换矩阵

};



#endif