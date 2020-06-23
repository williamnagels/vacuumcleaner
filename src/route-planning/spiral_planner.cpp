#include "spiral_planner.h"
#include <ros/console.h>
#include <math.h>
namespace
{
    Angle CalculateAngle(Coordinates b)
    {
        Angle angle = acos( b.x() / b.norm());

        if (b.y() < 0)
        {
            angle *= -1;
        }
        return angle;
    }
}

SpiralPlanner::SpiralPlanner(double radius)
    :_radius(radius)
{

}
Pose SpiralPlanner::GetNewPose()
{
    Coordinates current_goal_position = Calculate(_u);
    Coordinates future_goal_position = Calculate(_u + M_PI_2);

    ROS_INFO_STREAM("Current goal to: \"(" << current_goal_position.x() << ","<< current_goal_position.y()<<")\"");
    ROS_INFO_STREAM("Future goal to: \"(" << future_goal_position.x() << ","<< future_goal_position.y()<<")\"");

    _u += M_PI_2;
    Angle angle = CalculateAngle(future_goal_position - current_goal_position);
    ROS_INFO_STREAM("angle: \"(" << angle <<")\"");
    return Pose{current_goal_position, angle};
}

Coordinates SpiralPlanner::Calculate(double u) const
{
    return Coordinates{ (_radius / M_PI) * u * cos(-u), (_radius / M_PI) * u * sin(-u) };
}