#pragma once
#include "types.h"
#include "pose_generator.h"
struct SpiralPlanner : PoseGenerator
{
    SpiralPlanner(double radius, Radians goal_delta);

    Pose Generate() override;
    
    private:
        Radians _u = 0;
        double _radius = 0;
        Radians _goal_delta;
};