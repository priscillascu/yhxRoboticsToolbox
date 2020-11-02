#ifndef FORWARDKINE_HPP
#define FORWARDKINE_HPP

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include "BodyMotion.hpp"
#include "RobotBuild.hpp"

using namespace std;
using namespace Eigen;

Matrix4f FKTwist(RobotTwist Robot);

#endif