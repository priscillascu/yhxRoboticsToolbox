#include <iostream>
#include "EulerRot.hpp"

#define pi 3.141592653
using namespace std;

int main()
{
    Matrix<float, 3, 3> R1 = RotX(pi/2);
    cout << R1 << endl;
    Matrix<float, 3, 3> R2 = EulerRot("xyz", pi/2, pi/2, 0);
    cout << R2 << endl;
    return 0;
}