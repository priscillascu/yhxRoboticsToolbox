#include "BodyMotion.hpp"
#include <stdio.h>

Matrix<float, 3, 3> Skew(const Vector3f p)
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
        resultMat(3, 3) = 1;
    }
    return resultMat;
}

Matrix<float, 3, 3> RotMatExp(const Vector3f omega, float theta)
{
    // 若输入转轴不为单位向量，则转为单位向量1
    Vector3f omegaOne = omega/omega.norm();

    Matrix<float, 3, 3> RotMat;
    RotMat = Matrix3f::Identity() + sin(theta)*Skew(omegaOne) 
            + (1 - cos(theta))*(Skew(omegaOne)*Skew(omegaOne));
    return RotMat;
}

Vector3f GetRotAxis(const Matrix3f rotmat)
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

float GetRotTheta(const Matrix3f rotmat)
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

Matrix<float, 6, 6> AdjMapMat(const Matrix4f se3)
{
    Matrix3f rotTemp = se3.block<3, 3>(0, 0);  // 使用block来提取矩阵，注意括号后面是起始行和列
    Vector3f posTemp = se3.block<3, 1>(0, 3);
    Matrix3f transTemp = Skew(posTemp)*rotTemp;

    Matrix<float, 6, 6> resultMat;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            resultMat(i, j) = rotTemp(i, j);
            resultMat(i + 3, j) = transTemp(i, j);
            resultMat(i, j + 3) = 0;
            resultMat(i + 3, j + 3) = rotTemp(i, j);
        }
        
    }
    
    return resultMat;
}

Matrix4f SE3Twist(const VectorXf S, float theta)
{
    if(S.size() != 6)
    {
        cout << "Please enter 6 elements for the twist" << endl;
        return Matrix4f::Identity();
    }
    else
    {
        
        Vector3f omega;
        omega << S(0), S(1), S(2);
        Vector3f velocity;
        velocity << S(3), S(4), S(5);
        Matrix3f rotTemp;
        Vector3f posTemp;
        Matrix4f resultMat;

        if(omega.norm() == 0 && velocity.norm() == 0)
        {
            return Matrix4f::Identity();
        }
        if(omega.norm() == 0 && velocity.norm() != 0)
        {
            velocity = velocity/velocity.norm();
            rotTemp = Matrix3f::Identity();
            posTemp = theta*velocity;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    resultMat(i, j) = rotTemp(i, j);
                }
                resultMat(i, 3) = posTemp(i);
                resultMat(3, i) = 0;
            }
            resultMat(3, 3) = 1;

            return resultMat;
        }
        if(omega.norm() != 1)
        {
            omega = omega/omega.norm();
            velocity = velocity/velocity.norm();
            rotTemp = RotMatExp(omega, theta);
            posTemp = (Matrix3f::Identity()*theta + (1 - cos(theta))*Skew(omega)
                     + (theta - sin(theta))*Skew(omega)*Skew(omega))*velocity;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    resultMat(i, j) = rotTemp(i, j);
                }
                resultMat(i, 3) = posTemp(i);
                resultMat(3, i) = 0;
            }
            resultMat(3, 3) = 1;
            
            return resultMat;
        }
    }
    
}

VectorXf GetTwist(const Matrix4f se3)
{
    Matrix3f rotTemp = se3.block<3, 3>(0, 0);  // 使用block来提取矩阵，注意括号后面是起始行和列
    Vector3f posTemp = se3.block<3, 1>(0, 3);

    VectorXf resultTwist(6);
    if(rotTemp == Matrix3f::Identity())
    {
        resultTwist << 0, 0, 0, posTemp/posTemp.norm();
        return resultTwist;
    }
    else
    {
        resultTwist << GetRotAxis(rotTemp), (1/GetRotTheta(rotTemp)*Matrix3f::Identity() 
                    - 0.5*Skew(GetRotAxis(rotTemp)) + ((1/GetRotTheta(rotTemp) 
                    - 0.5*1/tan(GetRotTheta(rotTemp)/2)))*Skew(GetRotAxis(rotTemp))*Skew(GetRotAxis(rotTemp)))*posTemp;
        return resultTwist;
    }
    
}

float GetTwistTheta(const Matrix4f se3)
{
    Matrix3f rotTemp = se3.block<3, 3>(0, 0);

    if(rotTemp == Matrix3f::Identity())
    {
        return 0;
    }
    else
    {
        return GetRotTheta(rotTemp);
    }
    
}
