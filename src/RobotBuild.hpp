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

const float g = 9.81;  // 重力加速度

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

    /*
    机器人基础信息，名字，自由度
    */
    string robotName;
    int DoF;

    /*
    初始化有数据的机器人信息，如果自定义机器人，则不需要调用此函数
    */
    void RobotTwistInit(int dof);

    /*
    关节旋量信息
    */
    VectorXf theta;   // 各个关节的旋转角度
    VectorXf omega;   // 各个关节的旋量的角速度轴
    VectorXf qAxis;  // 各个关节的旋量的轴线上任一点
    VectorXf velocity;  // 各个关节的旋量的线速度分量

    /*
    末端执行器位姿
    */
    Matrix4f initSE3;  // 初始末端相对于基座标系的齐次变换矩阵

    /*
    动力学参数
    */
    VectorXf mass;  // 各个连杆的质量
    Matrix<float, Dynamic, 3> Inertia;  // 各个连杆的转动惯量矩阵，维度为3*3n
    VectorXf dTheta;  // 每个关节的角速度
    VectorXf ddTheta;  // 每个关节的角加速度

};



#endif