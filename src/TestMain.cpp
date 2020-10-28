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

    string name = "234";
    
    Matrix4f R2 = SE3Twist(v, th1);
    
    cout << "R2 = \n" << R2 << endl;
    cout << v << "\t" << th1 << endl;
    
    RobotDH p560("PUMA560");
    cout << p560.robotName << "\t" << p560.DoF << "\n" << p560.theta << endl;
    
    return 0;
}