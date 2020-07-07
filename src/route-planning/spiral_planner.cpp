#include "parameter.h"
#include "spiral_planner.h"
#include <ros/console.h>
#include <math.h>


SpiralPlanner::SpiralPlanner()
    :_radius(GetParameter(PARAM_ROBOT_RADIUS, PARAM_DEFAULT_ROBOT_RADIUS))
    ,_goal_delta(M_PI/GetParameter(PARAM_SPIRAL_DELTA_DENOMINATOR,PARAM_DEFAULT_SPIRAL_DELTA_DENOMINATOR))
{
}

Pose SpiralPlanner::Generate()
{
    Coordinates current_goal_position = Coordinates{ (_radius / M_PI) * _u * cos(-_u), (_radius / M_PI) * _u * sin(-_u) };
    _u += _goal_delta;
    return Pose{current_goal_position, 0.0};
}