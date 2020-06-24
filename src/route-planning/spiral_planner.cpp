#include "spiral_planner.h"
#include <ros/console.h>
#include <math.h>


SpiralPlanner::SpiralPlanner(double radius, Radians goal_delta)
    :_radius(radius)
    ,_goal_delta(goal_delta)
{

}
Pose SpiralPlanner::GetNewPose()
{
    Coordinates current_goal_position = Coordinates{ (_radius / M_PI) * _u * cos(-_u), (_radius / M_PI) * _u * sin(-_u) };
    _u += _goal_delta;

    ROS_DEBUG_STREAM("Current goal to: \"(" << current_goal_position.x() << ","<< current_goal_position.y()<<")\"");   
    return Pose{current_goal_position, 0.0}; //rotation is ignored by move base; cfr: base_local_planner_params.yaml
}