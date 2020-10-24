#include <iostream>
#include "BodyMotion.hpp"

using namespace std;

int main()
{
    Matrix<float, 3, 3> R1 = RotX(pi/2);
    cout << R1 << endl;
    Matrix<float, 3, 3> R2 = EulerRot("xyz", pi/2, pi/2, 0);
    cout << R2 << endl;

    Matrix<float, 3, 3> R3 = SE2(1, 2, 30*pi/180);
    Matrix<float, 3, 3> R4 = SE2(2, 1, 0);
    cout << R3*R4 << endl;

    Matrix<float, 4, 4> R5 = SE3(1.23, 2.3, -1.67, "xyz", pi/6, -pi/5, pi/2);
    cout << R5 << endl;

    Matrix3f R6 = Skew(Vector3f(1, 2, 3));
    cout << R6 << endl;

    Vector3f omega(1, 2, 3);
    cout << R2*Skew(omega)*R2.transpose() << endl;
    cout << Skew(R2*omega) << endl;

    Matrix3f R7 = Matrix3f::Identity();
    cout << R7 << endl;

    Matrix3f R8 = RotMatExp(Vector3f(0, 0.866, 0.5), 30*pi/180);
    cout << R8 << endl;

    Vector3f Axis1 = GetRotAxis(R8);
    cout << Axis1 << endl;
    float theta = GetRotTheta(R8);
    cout << theta << endl;
    Matrix3f R9 = RotMatExp(Axis1, theta);
    cout << R9 << endl;
    return 0;
}