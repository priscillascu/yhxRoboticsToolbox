#include <iostream>
#include "BodyMotion.hpp"

using namespace std;

int main()
{
    Matrix4f R1 = SE3(1, 2, 3, "xyz", pi/5, pi/3, pi/4);
    cout << "R1 = " << R1 << endl;

    VectorXf v(6);
    v << 1, 0, 0, 0, 0, 0;
    Matrix4f R2 = SE3Twist(v, pi/2);
    cout << "R2 = " << R2 << endl;
    return 0;
}