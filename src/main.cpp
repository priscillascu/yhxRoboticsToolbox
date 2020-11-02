#include <iostream>
#include "BodyMotion.hpp"
#include "RobotBuild.hpp"

using namespace std;

int main()
{
    Matrix4f R1 = SE3(1, 2, 3, "xyz", pi/5, pi/3, pi/4);
    cout << "R1 = \n" << R1 << endl;
    
    VectorXf v = GetTwist(R1);
    float th1 = GetTwistTheta(R1);
    cout << v << "\t" << th1 << endl;

    Matrix4f R2 = SE3Twist(v, th1);
    
    cout << "R2 = \n" << R2 << endl;
    cout << v << "\t" << th1 << endl;
    
    RobotDH *p560 = new RobotDH("PUMA560");
    p560->RobotDHInit();

    cout << p560->robotName << endl;
    cout << p560->alpha << endl;
    
    RobotTwist *UR5 = new RobotTwist("UR5");
    UR5->RobotTwistInit();  // 此时会提示请自行创建
    // UR5连杆数据，用于创建旋量
    float W1 = 0.109;
    float W2 = 0.082;
    float L1 = 0.425;
    float L2 = 0.392;
    float H1 = 0.089;
    float H2 = 0.095;
    UR5->DoF = 6;
    UR5->initSE3 << -1, 0, 0, L1 + L2,
                    0, 0, 1, W1 + W2,
                    0, 1, 0, H1 - H2,
                    0, 0, 0, 1;
    UR5->omega.resize(3*UR5->DoF);
    UR5->omega << 0, 0, 1,
                0, 1, 0,
                0, 1, 0, 
                0, 1, 0, 
                0, 0, -1, 
                0, 1, 0;
    UR5->velocity.resize(3*UR5->DoF);
    UR5->velocity << 0, 0, 0, 
                -H1, 0, 0, 
                -H1, 0, L1, 
                -H1, 0, L1+L2, 
                -W1, L1+L2, 0, 
                H2-H1, 0, L1+L2;
    UR5->theta.resize(UR5->DoF);
    UR5->theta << 0, -pi/2, 0, 0, pi/2, 0;
    cout << UR5->omega << "\n" << UR5->velocity << endl;
    cout << UR5->FKTwist() << endl;

    delete p560;
    delete UR5;
    return 0;
}