#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <iostream>
#include <cmath>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include "BodyMotion.hpp"
#include "RobotBuild.hpp"

using namespace std;
using namespace Eigen;

/*
机器人运动学，包括正、逆、微分(雅克比)
*/

// 都是以传引用的方式传值，因为eigen有些操作不能用const，所以没有传入常值引用
Matrix4f FKSpace(RobotTwist &robot);  // 利用在基座标系下表示的旋量求正运动学
Matrix4f FKBody(RobotTwist &robot);   // 虽然机器人的旋量都是在基座标系中表示，还是可以转为末端坐标系下，然后再求正运动学
Matrix<float, 6, 6> JacobianSpace(RobotTwist &robot);  // 机器人的空间雅克比，各列为各个关节的旋量在基座标系下的表示
Matrix<float, 6, 6> JacobianBody(RobotTwist &robot);   // 机器人的物体雅克比，各列为各个关节的旋量在末端的表示
VectorXf IKNewton(RobotTwist &robot, const Matrix4f Target, VectorXf *initAng);   // 基于牛顿-拉夫森法的逆运动学求解，输入机器人对象，目标齐次变换矩阵

#endif