#pragma once
#include <Eigen/Dense>

using Coordinates = Eigen::Vector2d;
using Radians = double;

struct Pose
{
    Coordinates coordinates; // x, y coordinates
    Radians angle; //rotation z-axis
};
