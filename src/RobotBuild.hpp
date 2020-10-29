#ifndef ROBOTBUILD_HPP
#define ROBOTBUILD_HPP

#include <iostream>
#include <cmath>
#include <string>
#include <unordered_map>
#include "BodyMotion.hpp"

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


#endif