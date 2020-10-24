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

#define pi 3.141592653

using namespace std;
using namespace Eigen;

// 由三维向量求得对应的反对称矩阵(叉乘矩阵)
Matrix<float, 3, 3> Skew(Vector3f p);

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

// 基于Rodrigues' formula的旋转矩阵求解，输入旋转轴和旋转角度
Matrix<float, 3, 3> RotMatExp(Vector3f omega, float theta = 0);

// 由旋转矩阵反解出空间旋转轴，矩阵对数
Vector3f GetRotAxis(Matrix3f rotmat);

// 由旋转矩阵反解出绕空间中单轴旋转的角度，矩阵对数
float GetRotTheta(Matrix3f rotmat);

#endif