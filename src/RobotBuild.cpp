#include "RobotBuild.hpp"

// 有参构造，根据机器人型号来给DH参数赋初值
RobotDH::RobotDH(string robot)
{
    robotName = robot;
}
// 默认是puma560机械臂，机器人型号用大写
RobotDH::RobotDH() 
{
    robotName = "PUMA560";
}

RobotDH::~RobotDH() {}

// 机器人DH参数初始化
void RobotDH::RobotDHInit()
{
    string robot = robotName;
    // 构造哈希表，建立机器人型号和数字的对应关系
    unordered_map<string, int> robotType = 
    {
        {"PUMA560", 1},
    };
    // 根据输入的机器人名字来选择对应的机器人数据
    unordered_map<string, int>::const_iterator type = robotType.find(robot);
    if(type == robotType.end())
    {
        cout << "Not find this robot, please define it yourself" << endl;
        robotName = robot;
    }
    else
    {
        switch (type->second)
        {
        case 1:
            robotName = "PUMA560";
            DoF = 6;
            theta.resize(DoF);
            d.resize(DoF);
            a.resize(DoF);
            alpha.resize(DoF);

            theta << 0, 0, 0, 0, 0, 0;
            d << 0, 0, 0.15005, 0.4318, 0, 0;
            a << 0, 0.4318, 0.0203, 0, 0, 0;
            alpha << pi/2, 0, -pi/2, pi/2, -pi/2, 0;

            break;
        
        default:
            break;
        }

        cout << "Robot built successed!" << endl;
        cout << "**********************************" << endl;
        cout << "**Robot Name:\t" << robotName << "\t**" << endl;
        cout << "**DH-d:**" << endl;
        cout << a << endl;
        cout << "**DH-a:**" << endl;
        cout << d << endl;
        cout << "**DH-alpha:**" << endl;
        cout << alpha<< endl;
    }
}



RobotTwist::RobotTwist(string robot)
{
    robotName = robot;
}

RobotTwist::RobotTwist()
{
    robotName = "CRP14";
}

RobotTwist::~RobotTwist()
{
}

void RobotTwist::RobotTwistInit()
{
    string robot = robotName;
    // 构造哈希表，建立机器人型号和数字的对应关系
    unordered_map<string, int> robotType = 
    {
        {"CRP14", 1},
    };
    // 根据输入的机器人名字来选择对应的机器人数据
    unordered_map<string, int>::const_iterator type = robotType.find(robot);
    if(type == robotType.end())
    {
        cout << "Not find this robot, please define it yourself" << endl;
        robotName = robot;
    }
    else
    {
        switch (type->second)
        {
        case 1:
            robotName = "CRP14";
            DoF = 6;
            theta.resize(DoF);
            omega.resize(3*DoF);

            theta << 0, 0, 0, 0, 0, 0;
            // twist << 

            break;
        
        default:
            break;
        }

        cout << "Robot built successed!" << endl;
        cout << "**********************************" << endl;
        cout << "**Robot Name:\t" << robotName << "\t**" << endl;
        cout << "**DH-d:**" << endl;
    }
}

Matrix4f RobotTwist::FKTwist()
{
    VectorXf S(6);
    Matrix4f tempSE3;
    tempSE3 = initSE3;
    for (int i = 0; i < DoF; ++i)
    {
        // 当角度为０时，该角度对应的旋量齐次变换矩阵为单位阵，无需再计算
        if(theta(DoF - 1 - i) == 0)
        {
            continue;
        }
        // 当DoF=3时，依次按照{0, 1, 2}, {3, 4, 5}, {6, 7, 8}的顺序存放，即3*(DoF - 1 - i)
        // vector不能一次性截取赋值，需要使用segment(i, j)函数，截取从i开始的j个元素
        S << omega.segment(3*(DoF - 1 - i), 3), velocity.segment(3*(DoF - 1 - i), 3);
        tempSE3 = SE3Twist(S, theta(DoF - 1 - i))*tempSE3;
    }
    return tempSE3;
}