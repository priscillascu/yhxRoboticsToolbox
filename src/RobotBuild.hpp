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
    机器人运动学，包括正、逆、微分(雅克比)
    */
    
    Matrix4f FKSpace();  // 利用在基座标系下表示的旋量求正运动学
    Matrix4f FKBody();   // 虽然机器人的旋量都是在基座标系中表示，还是可以转为末端坐标系下，然后再求正运动学
    Matrix<float, 6, 6> JacobianSpace();  // 机器人的空间雅克比，各列为各个关节的旋量在基座标系下的表示
    Matrix<float, 6, 6> JacobianBody();   // 机器人的物体雅克比，各列为各个关节的旋量在末端的表示

    /*
    关节旋量信息
    */
    VectorXf theta;   // 各个关节的旋转角度
    VectorXf omega;   // 各个关节的旋量的角速度轴
    VectorXf qAxis;  // 各个关节的旋量的线速度分量
    VectorXf velocity;

    /*
    末端执行器位姿
    */
    Matrix4f initSE3;  // 初始末端相对于基座标系的齐次变换矩阵

};



#endif