#include "EulerRot.hpp"

Matrix<float, 3, 3> RotX(float x)
{
    static Matrix<float, 3, 3> rotx;  // 在函数中使用的new，在调用完该函数之后，在调用的地方释放相应地址
    rotx << 1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x);
    return rotx;
}

Matrix<float, 3, 3> RotY(float y)
{
    static Matrix<float, 3, 3> roty;
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