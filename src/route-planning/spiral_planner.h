#pragma once
#include "types.h"
#include "pose_generator.h"
struct SpiralPlanner : PoseGenerator
{
    SpiralPlanner();
    Pose Generate() override;
    
    private:
        Radians _u = 0;
        double _radius;
        Radians _goal_delta;
};