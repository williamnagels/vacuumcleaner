#include "spiral_planner.h"
#include <ros/console.h>
#include <math.h>
namespace
{
    Radians CalculateAngle(Coordinates b)
    {
        Radians angle = acos( b.x() / b.norm());
        angle -= 2 * angle * (b.y() <0);
        return angle;
    }
}

SpiralPlanner::SpiralPlanner(double radius, Radians goal_delta)
    :_radius(radius)
    ,_goal_delta(goal_delta)
{

}
Pose SpiralPlanner::GetNewPose()
{
    Radians next_u = _u + _goal_delta;
    Coordinates current_goal_position = Calculate(_u);
    Coordinates future_goal_position = Calculate(next_u);
    Radians angle = CalculateAngle(future_goal_position - current_goal_position);
    _u = next_u;

    ROS_DEBUG_STREAM("Current goal to: \"(" << current_goal_position.x() << ","<< current_goal_position.y()<<")\"");
    ROS_DEBUG_STREAM("Future goal to: \"(" << future_goal_position.x() << ","<< future_goal_position.y()<<")\"");
    ROS_DEBUG_STREAM("Angle (radians): \"(" << angle <<")\"");
    
    return Pose{current_goal_position, angle};
}

Coordinates SpiralPlanner::Calculate(double u) const
{
    return Coordinates{ (_radius / M_PI) * u * cos(-u), (_radius / M_PI) * u * sin(-u) };
}