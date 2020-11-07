#include <iostream>
#include "BodyMotion.hpp"
#include "RobotBuild.hpp"
#include "Kinematics.hpp"

using namespace std;

int main()
{   
    RobotTwist *p560 = new RobotTwist("PUMA560");
    p560->RobotTwistInit(6);

    /*
    ************************正运动学验证**********************************
    选取三组不同的关节角，与Matlab对比，验证了机械臂旋量模型、正运动学算法正确性
    */
    p560->theta << 0, 0, 0, 0, 0, 0;
    cout << "Space Forward Kinematics at qz is \n" << FKSpace(*p560) << endl;
    cout << "Body Forward Kinematics at qz is \n" << FKBody(*p560) << endl;

    p560->theta << 0, pi/2, -pi/2, 0, 0, 0;
    cout << "Space Forward Kinematics at qr is \n" << FKSpace(*p560) << endl;
    cout << "Body Forward Kinematics at qr is \n" << FKBody(*p560) << endl;

    p560->theta << 0, pi/4, -pi, 0, pi/4, 0;
    cout << "Space Forward Kinematics at qn is \n" << FKSpace(*p560) << endl;
    cout << "Body Forward Kinematics at qn is \n" << FKBody(*p560) << endl;

    /*
    ************************雅可比验证**********************************
    选取三组不同的关节角，与Matlab对比，验证了机械臂旋量模型、雅可比算法正确性
    */
    p560->theta << 0, 0, 0, 0, 0, 0;
    cout << "Space Jacobian at qz is \n" << JacobianSpace(*p560) << endl;
    cout << "Body Jacobian at qz is \n" << JacobianBody(*p560) << endl;

    p560->theta << 0, pi/2, -pi/2, 0, 0, 0;
    cout << "Space Jacobian at qr is \n" << JacobianSpace(*p560) << endl;
    cout << "Body Jacobian at qr is \n" << JacobianBody(*p560) << endl;
    

    p560->theta << 0, pi/4, -pi, 0, pi/4, 0;
    cout << "Space Jacobian at qn is \n" << JacobianSpace(*p560) << endl;
    cout << "Body Jacobian at qn is \n" << JacobianBody(*p560) << endl;

    /*
    ************************逆运动学验证**********************************
    逆运动学验证，多组位姿，注意需要避开奇异点，且初始角度不能相差太大，<0.5rad
    */
    VectorXf initAng(6);
    initAng << 0.1, 0.5, 0, 0.1, 0.5, 0.1;
    Matrix4f target = FKSpace(*p560);
    cout << "Inverse Kinematics is " << IKNewton(*p560, target, initAng) << endl;

    p560->theta << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    target = FKSpace(*p560);
    initAng << 0, 0, 0.5, 0.5, 0.5, 0.5;
    cout << "Inverse Kinematics is " << IKNewton(*p560, target, initAng) << endl;
    
    delete p560;
    return 0;
}
