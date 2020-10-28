#include "RobotBuild.hpp"

// 有参构造，根据机器人型号来给DH参数赋初值，默认是puma560机械臂，机器人型号用大写
RobotDH::RobotDH(string robot)
{
    // 构造哈希表，建立机器人型号和数字的对应关系
    unordered_map<string, int> robotType = 
    {
        {"PUMA560", 1},
    };
    // 根据输入的机器人名字来选择对应的机器人数据
    unordered_map<string, int>::const_iterator type = robotType.find(robot);
    if(type == robotType.end())
    {
        cout << "Not find this robot" << endl;
        robotName = "Not Defined";
        DoF = 1;
        theta.resize(DoF);
        // d.resize(DoF);
        // a.resize(DoF);
        // alpha.resize(DoF);

        theta(0) = 0;
        // d << 0;
        // a << 0;
        // alpha << 0;
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
            theta(0) = 0;
            theta(1) = 0;
            theta(2) = 0;
            theta(3) = 0;
            theta(4) = 0;
            theta(5) = 0;

            
        
            cout << "PUMA560 built successed!" << endl;
            break;
        
        default:
            break;
        }
    }
}

RobotDH::RobotDH()
{
    cout << "Call for constructor" << endl;
}

RobotDH::~RobotDH()
{
    cout << "Call for distructor" << endl;
}