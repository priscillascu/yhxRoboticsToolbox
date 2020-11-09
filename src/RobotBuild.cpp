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

void RobotTwist::RobotTwistInit(int dof)
{
    DoF = dof;
    omega.resize(3*DoF);  // 对于可变长度的，需要先声明长度再赋值
    qAxis.resize(3*DoF);
    velocity.resize(3*DoF);
    theta.resize(DoF);
    string robot = robotName;
    // 构造哈希表，建立机器人型号和数字的对应关系
    unordered_map<string, int> robotType = 
    {
        {"CRP14", 1},
        {"PUMA560", 2}
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
        {
            robotName = "CRP14";
            DoF = 6;
            theta.resize(DoF);
            omega.resize(3*DoF);
            qAxis.resize(3*DoF);

            theta << 0, 0, 0, 0, 0, 0;
            omega << 0, 0, 1,
                    0, -1, 0,
                    0, -1, 0,
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, -1;
            qAxis <<  0, 0, 0,
                    169.9876, 0, -0.0121,
                    170.0136, 0, 613.9879,
                    0, -0.0112, 814.0441,
                    810.0109, 0, 814.0039,
                    810.0018, 0, 0;
            break;
        }

        case 2:
        {
            robotName = "PUMA560";
            DoF = 6;
            theta.resize(DoF);
            omega.resize(3*DoF);
            qAxis.resize(3*DoF);
            velocity.resize(3*DoF);

            mass.resize(DoF);
            Inertia.resize(3*DoF, 3);
            dTheta.resize(DoF);
            ddTheta.resize(DoF);

            theta << 0, 0, 0, 0, 0, 0;
            omega << 0, 0, 1,
                    0, -1, 0,
                    0, -1, 0,
                    0, 0, 1,
                    0, -1, 0,
                    0, 0, 1;
            qAxis <<  0, 0, 0,
                    0, 0, 0,
                    431.8, 0, 0,
                    452.1, -150, 0,
                    452.1, -150, 431.8,
                    452.1, -150, 431.8;
            initSE3 << 1, 0, 0, 452.1,
                        0, 1, 0, -150,
                        0, 0, 1, 431.8,
                        0, 0, 0, 1;

            mass << 0, 17.4, 4.8, 0.82, 0.34, 0.09;
            Inertia << 0, 0, 0, 
                        0, 0.35, 0,
                        0, 0, 0,
                        0.13, 0, 0,
                        0, 0.524, 0,
                        0, 0, 0.539,
                        0.066, 0, 0,
                        0, 0.086, 0,
                        0, 0, 0.0125,
                        0.0018, 0, 0,
                        0, 0.0013, 0,
                        0, 0, 0.0018,
                        0.0003, 0, 0,
                        0, 0.0004, 0,
                        0, 0, 0.0003,
                        0.00015, 0, 0,
                        0, 0.00015, 0,
                        0, 0, 0.00004;
            dTheta << 0, 0, 0, 0, 0, 0;
            ddTheta << 0, 0, 0, 0, 0, 0;
            break;
        }  
        
        default:
            break;
        }

        cout << "Robot built successed!" << endl;
        cout << "**********************************" << endl;
        cout << "**Robot Name:\t" << robotName << "\t**" << endl;
    }
}

