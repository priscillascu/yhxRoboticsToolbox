#ifndef EULERROTMAT_HPP
#define EULERROTMAT_HPP

#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <cmath>
#include <unordered_map>

using namespace std;
using namespace Eigen;

// 单轴旋转矩阵
Matrix<float, 3, 3> RotX(float x);
Matrix<float, 3, 3> RotY(float y);
Matrix<float, 3, 3> RotZ(float z);

Matrix<float, 3, 3> EulerRot(string rotOrder, float parameter1 = 0, float parameter2 = 0, float parameter3 = 0);


#endif