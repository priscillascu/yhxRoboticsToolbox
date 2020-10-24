#include "BodyMotion.hpp"

Matrix<float, 3, 3> Skew(Vector3f p)
{
    Matrix<float, 3, 3> resultMat;
    resultMat << 0, -p(2), p(1),
                p(2), 0, -p(0),
                -p(1), p(0), 0;
    return resultMat;
}

Matrix<float, 3, 3> RotX(float x)
{
    Matrix<float, 3, 3> rotx; 
    rotx << 1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x);
    return rotx;
}

Matrix<float, 3, 3> RotY(float y)
{
    Matrix<float, 3, 3> roty;
    roty << cos(y), 0, sin(y),
            0, 1, 0,
            -sin(y), 0, cos(y);
    return roty;
}

Matrix<float, 3, 3> RotZ(float z)
{
    Matrix<float, 3, 3> rotz;
    rotz << cos(z), -sin(z), 0,
            sin(z), cos(z), 0,
            0, 0, 1;
    return rotz;
}

Matrix<float, 3, 3> EulerRot(string rotOrder, float parameter1, float parameter2, float parameter3)
{
    // 构造哈希表，建立旋转顺序和数字的对应关系
    unordered_map<string, int> rotMethod = 
    {
        {"xyz", 1},
        {"xzy", 2},
        {"yxz", 3},
        {"yzx", 4},
        {"zxy", 5},
        {"zyx", 6},
        {"XYZ", 7},
        {"XZY", 8},
        {"YXZ", 9},
        {"YZX", 10},
        {"ZXY", 11},
        {"ZYX", 12}
    };
    // 根据输入的旋转方式对应的数字来进行矩阵乘法
    unordered_map<string, int>::const_iterator pos = rotMethod.find(rotOrder);

    Matrix<float, 3, 3> rotMat;
    if(pos == rotMethod.end())
    {
        cout << "Please enter the rotate order!" << endl;
        rotMat << 0, 0, 0,
                  0, 0, 0,
                  0, 0, 0;
    }
    else
    {
        switch (pos->second)
        {
            // 两个条件或的case
            case 1:
            case 7:
            {
                rotMat = RotX(parameter1)*RotY(parameter2)*RotZ(parameter3);
                break;
            }
            case 2:
            case 8:
            {
                rotMat = RotX(parameter1)*RotZ(parameter2)*RotY(parameter3);
                break;
            }
            case 3:
            case 9:
            {
                rotMat = RotY(parameter1)*RotX(parameter2)*RotZ(parameter3);
                break;
            }
            case 4:
            case 10:
            {
                rotMat = RotY(parameter1)*RotZ(parameter2)*RotX(parameter3);
                break;
            }
            case 5:
            case 11:
            {
                rotMat = RotZ(parameter1)*RotX(parameter2)*RotY(parameter3);
                break;
            }
            case 6:
            case 12:
            {
                rotMat = RotZ(parameter1)*RotY(parameter2)*RotX(parameter3);
                break;
            }
        }
    }   
    return rotMat;
}

Matrix<float, 3, 3> SE2(float x, float y, float theta)
{
    Matrix<float, 3, 3> resultMat;
    resultMat << cos(theta), -sin(theta), x,
                sin(theta), cos(theta), y,
                0,          0,          1;
    return resultMat;
}

Matrix<float, 4, 4> SE3(float x, float y, float z, string strRotOrder, 
float parameter1, float parameter2, float parameter3)
{
    float tempPos[3] = {x, y, z};
    Matrix<float, 3, 3> tempRot = EulerRot(strRotOrder, parameter1, parameter2, parameter3);
    Matrix<float, 4, 4> resultMat;
    resultMat.Identity();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            resultMat(i, j) = tempRot(i, j);
        }
        resultMat(i, 3) = tempPos[i];
    }
    return resultMat;
}

Matrix<float, 3, 3> RotMatExp(Vector3f omega, float theta)
{
    Matrix<float, 3, 3> RotMat;
    RotMat = Matrix3f::Identity() + sin(theta)*Skew(omega) + (1 - cos(theta))*(Skew(omega)*Skew(omega));
    return RotMat;
}

Vector3f GetRotAxis(Matrix3f rotmat)
{
    if(rotmat == Matrix3f::Identity())
    {
        cout << "Rotate Axis is Undefined" << endl;
        return Vector3f(0, 0, 0);
    }
    else if(rotmat.trace() == 1)
    {
        return 1/sqrt(2*(1+rotmat(2, 2)))*Vector3f(rotmat(0, 2), rotmat(1, 2), 1 + rotmat(2, 2));
    }
    else
    {
        Matrix3f temp = 1/(2*sin(acos(0.5*(rotmat.trace() - 1))))*(rotmat - rotmat.transpose());
        return Vector3f(-temp(1, 2), temp(0, 2), -temp(0, 1));
    }
}

float GetRotTheta(Matrix3f rotmat)
{
    if(rotmat == Matrix3f::Identity())
    {
        cout << "No rotation happened" << endl;
        return 0;
    }
    else if(rotmat.trace() == 1)
    {
        return pi;
    }
    else
    {
        return acos(0.5*(rotmat.trace() - 1));
    }
}
