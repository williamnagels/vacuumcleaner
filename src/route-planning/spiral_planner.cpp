#include "spiral_planner.h"
#include <ros/console.h>
#include <math.h>


SpiralPlanner::SpiralPlanner(double radius, Radians goal_delta)
    :_radius(radius)
    ,_goal_delta(goal_delta)
{

}
std::queue<Pose> SpiralPlanner::GetNextPoses()
{
    std::queue<Pose> poses;
    int64_t poses_to_generate = 100;

    while (poses_to_generate-- > 0)
    {
        Coordinates current_goal_position = Coordinates{ (_radius / M_PI) * _u * cos(-_u), (_radius / M_PI) * _u * sin(-_u) };
        _u += _goal_delta;
        poses.push(Pose{current_goal_position, 0.0});
    }

    return poses;
}