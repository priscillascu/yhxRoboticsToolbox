#include "Kinematics.hpp"

Matrix4f FKSpace(RobotTwist &robot)
{   
    VectorXf S(6);
    Matrix4f tempSE3;
    tempSE3 = robot.initSE3;
    for (int i = 0; i < robot.DoF; ++i)
    {
        robot.velocity.segment(3*i, 3) = Skew(robot.omega.segment(3*i, 3))*robot.qAxis.segment(3*i, 3);
        // 当角度为０时，该角度对应的旋量齐次变换矩阵为单位阵，无需再计算
        if(robot.theta(robot.DoF - 1 - i) == 0)
        {
            continue;
        }
        // 当DoF=3时，依次按照{0, 1, 2}, {3, 4, 5}, {6, 7, 8}的顺序存放，即3*(DoF - 1 - i)
        // vector不能一次性截取赋值，需要使用segment(i, j)函数，截取从i开始的j个元素
        S << robot.omega.segment(3*(robot.DoF - 1 - i), 3), robot.velocity.segment(3*(robot.DoF - 1 - i), 3);
        tempSE3 = SE3Twist(S, robot.theta(robot.DoF - 1 - i))*tempSE3;
    }
    return tempSE3;
}

Matrix4f FKBody(RobotTwist &robot)
{
    

    VectorXf S(6);
    Matrix4f tempSE3;
    tempSE3 = robot.initSE3;
    Matrix<float, 6, 6> AdM = AdjMapMat(robot.initSE3.inverse());
    for (int i = 0; i < robot.DoF; ++i)
    {
        robot.velocity.segment(3*i, 3) = Skew(robot.omega.segment(3*i, 3))*robot.qAxis.segment(3*i, 3);
        // 当角度为０时，该角度对应的旋量齐次变换矩阵为单位阵，无需再计算
        if(robot.theta(i) == 0)
        {
            continue;
        }
        // 当DoF=3时，依次按照{0, 1, 2}, {3, 4, 5}, {6, 7, 8}的顺序存放
        // vector不能一次性截取赋值，需要使用segment(i, j)函数，截取从i开始的j个元素
        S << robot.omega.segment(3*i, 3), robot.velocity.segment(3*i, 3);
        S = AdM*S;
        tempSE3 = tempSE3*SE3Twist(S, robot.theta(i));
    }
    return tempSE3;
}

Matrix<float, 6, 6> JacobianSpace(RobotTwist &robot)
{
    

    VectorXf curS(6);
    VectorXf preS(6);

    Matrix<float, 6, 6> resultMat;
    resultMat = Matrix<float, 6, 6>::Zero();

    Matrix4f tempT = Matrix4f::Identity();

    VectorXf Js(6);
    // Js = VectorXf::Zero();  // 请勿使用Zero给向量赋初值，只能给静态向量赋初值

    for (int i = 0; i < robot.DoF; ++i)
    {
        robot.velocity.segment(3*i, 3) = Skew(robot.omega.segment(3*i, 3))*robot.qAxis.segment(3*i, 3);
        preS = curS;
        curS << robot.omega.segment(3*i, 3), robot.velocity.segment(3*i, 3);
        

        if(i == 0)
        {
            Js = curS;
        }
        else
        {
            tempT = tempT*SE3Twist(preS, robot.theta(i - 1));
            Js = AdjMapMat(tempT)*curS;
        }

        resultMat.block<6, 1>(0, i) = Js; 
    }
    return resultMat;
}

Matrix<float, 6, 6> JacobianBody(RobotTwist &robot)
{
    Matrix<float, 6, 6> AdM = AdjMapMat(robot.initSE3.inverse());
    
    VectorXf curS(6);
    VectorXf preS(6);

    Matrix4f tempT = Matrix4f::Identity();
    
    Matrix<float, 6, 6> resultMat;
    resultMat = Matrix<float, 6, 6>::Zero();

    VectorXf Js(6);
    // Js = VectorXf::Zero();  // 请勿使用Zero给向量赋初值，只能给静态向量赋初值

    for (int i = robot.DoF - 1; i >= 0; --i)
    {
        robot.velocity.segment(3*i, 3) = Skew(robot.omega.segment(3*i, 3))*robot.qAxis.segment(3*i, 3);
        preS = curS;
        curS << robot.omega.segment(3*i, 3), robot.velocity.segment(3*i, 3);
        curS = AdM*curS;

        if(i == robot.DoF - 1)
        {
            Js = curS;
        }
        else
        {
            tempT = tempT*SE3Twist(-preS, robot.theta(i + 1));
            Js = AdjMapMat(tempT) * curS;
        }

        resultMat.block<6, 1>(0, i) = Js; 
    }
    return resultMat;
}

VectorXf IKNewton(RobotTwist &robot, const Matrix4f Target, VectorXf *initAng)
{
    
}