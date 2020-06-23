#pragma once
#include <Eigen/Dense>

using Coordinates = Eigen::Vector2d;
using Angle = double;

struct Pose
{
    Coordinates coordinates;
    Angle angle;
};
