/*
角度均为弧度制
*/

#ifndef EULERROTMAT_HPP
#define EULERROTMAT_HPP

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <unordered_map>

using namespace std;
using namespace Eigen;

const double pi  = 3.141592653;
// 由三维向量求得对应的反对称矩阵(叉乘矩阵)
Matrix<float, 3, 3> Skew(const Vector3f p);

// 单轴旋转矩阵
Matrix<float, 3, 3> RotX(float x);
Matrix<float, 3, 3> RotY(float y);
Matrix<float, 3, 3> RotZ(float z);

// 欧拉角旋转矩阵
Matrix<float, 3, 3> EulerRot(string rotOrder, float parameter1 = 0, float parameter2 = 0, float parameter3 = 0);

// 二维欧几里得群，齐次矩阵，3x3
Matrix<float, 3, 3> SE2(float x = 0, float y = 0, float theta = 0);

// 三维欧几里得群，齐次矩阵，4x4
Matrix<float, 4, 4> SE3(float x = 0, float y = 0, float z = 0, string strRotOrder = "xyz", 
float parameter1 = 0, float parameter2 = 0, float parameter3 = 0);

// 基于Rodrigues' formula的旋转矩阵求解，输入旋转轴和旋转角度，旋转轴为单位轴
Matrix<float, 3, 3> RotMatExp(const Vector3f omega, float theta = 0);

// 由旋转矩阵反解出空间旋转轴，矩阵对数，结果为一个单位长度的旋转轴向量
Vector3f GetRotAxis(const Matrix3f rotmat);

// 由旋转矩阵反解出绕空间中单轴旋转的角度，矩阵对数
float GetRotTheta(const Matrix3f rotmat);

// 由SE3齐次变换矩阵求出伴随变换矩阵，用于运动旋量的坐标变换
Matrix<float, 6, 6> AdjMapMat(const Matrix4f se3);

// 使用螺旋轴求齐次变换矩阵SE3，螺旋轴为单位轴，输入螺旋轴和线速度组成的运动旋量以及旋转角度
Matrix4f SE3Twist(const VectorXf S, float theta);

// 由齐次变换矩阵SE3求出螺旋轴数据
VectorXf GetTwist(const Matrix4f se3);

// 求出与上一个螺旋轴对应的旋转角度
float GetTwistTheta(const Matrix4f se3);

#endif