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
                    170,0136, 0, 613.9879,
                    0, -0.0112, 814.0441,
                    810.0109, 0, 814.0039,
                    810.0018, 0, 0;

            break;
        
        default:
            break;
        }

        cout << "Robot built successed!" << endl;
        cout << "**********************************" << endl;
        cout << "**Robot Name:\t" << robotName << "\t**" << endl;
    }
}

Matrix4f RobotTwist::FKSpace()
{   
    VectorXf S(6);
    Matrix4f tempSE3;
    tempSE3 = initSE3;
    for (int i = 0; i < DoF; ++i)
    {
        velocity.segment(3*i, 3) = Skew(omega.segment(3*i, 3))*qAxis.segment(3*i, 3);
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

Matrix4f RobotTwist::FKBody()
{
    

    VectorXf S(6);
    Matrix4f tempSE3;
    tempSE3 = initSE3;
    Matrix<float, 6, 6> AdM = AdjMapMat(initSE3.inverse());
    for (int i = 0; i < DoF; ++i)
    {
        velocity.segment(3*i, 3) = Skew(omega.segment(3*i, 3))*qAxis.segment(3*i, 3);
        // 当角度为０时，该角度对应的旋量齐次变换矩阵为单位阵，无需再计算
        if(theta(i) == 0)
        {
            continue;
        }
        // 当DoF=3时，依次按照{0, 1, 2}, {3, 4, 5}, {6, 7, 8}的顺序存放
        // vector不能一次性截取赋值，需要使用segment(i, j)函数，截取从i开始的j个元素
        S << omega.segment(3*i, 3), velocity.segment(3*i, 3);
        S = AdM*S;
        tempSE3 = tempSE3*SE3Twist(S, theta(i));
    }
    return tempSE3;
}

Matrix<float, 6, 6> RobotTwist::JacobianSpace()
{
    

    VectorXf curS(6);
    VectorXf preS(6);

    Matrix<float, 6, 6> resultMat;
    resultMat = Matrix<float, 6, 6>::Zero();

    Matrix4f tempT = Matrix4f::Identity();

    VectorXf Js(6);
    // Js = VectorXf::Zero();  // 请勿使用Zero给向量赋初值，只能给静态向量赋初值

    for (int i = 0; i < DoF; ++i)
    {
        velocity.segment(3*i, 3) = Skew(omega.segment(3*i, 3))*qAxis.segment(3*i, 3);
        preS = curS;
        curS << omega.segment(3*i, 3), velocity.segment(3*i, 3);
        

        if(i == 0)
        {
            Js = curS;
        }
        else
        {
            tempT = tempT*SE3Twist(preS, theta(i - 1));
            Js = AdjMapMat(tempT)*curS;
        }

        resultMat.block<6, 1>(0, i) = Js; 
    }
    return resultMat;
}

Matrix<float, 6, 6> RobotTwist::JacobianBody()
{
    Matrix<float, 6, 6> AdM = AdjMapMat(initSE3.inverse());
    
    VectorXf curS(6);
    VectorXf preS(6);

    Matrix4f tempT = Matrix4f::Identity();
    
    Matrix<float, 6, 6> resultMat;
    resultMat = Matrix<float, 6, 6>::Zero();

    VectorXf Js(6);
    // Js = VectorXf::Zero();  // 请勿使用Zero给向量赋初值，只能给静态向量赋初值

    for (int i = DoF - 1; i >= 0; --i)
    {
        velocity.segment(3*i, 3) = Skew(omega.segment(3*i, 3))*qAxis.segment(3*i, 3);
        preS = curS;
        curS << omega.segment(3*i, 3), velocity.segment(3*i, 3);
        curS = AdM*curS;

        if(i == DoF - 1)
        {
            Js = curS;
        }
        else
        {
            tempT = tempT*SE3Twist(-preS, theta(i + 1));
            Js = AdjMapMat(tempT) * curS;
        }

        resultMat.block<6, 1>(0, i) = Js; 
    }
    return resultMat;
}