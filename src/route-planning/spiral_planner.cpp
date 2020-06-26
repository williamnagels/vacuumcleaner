#include "spiral_planner.h"
#include <ros/console.h>
#include <math.h>


SpiralPlanner::SpiralPlanner(double radius, Radians goal_delta)
    :_radius(radius)
    ,_goal_delta(goal_delta)
{
}

Pose SpiralPlanner::Generate()
{
    Coordinates current_goal_position = Coordinates{ (_radius / M_PI) * _u * cos(-_u), (_radius / M_PI) * _u * sin(-_u) };
    _u += _goal_delta;
    return Pose{current_goal_position, 0.0};
}