#include "types.h"
#include <queue>
struct SpiralPlanner
{
    SpiralPlanner(double radius, Radians goal_delta);

    std::queue<Pose> GetNextPoses();
    
    private:
        Radians _u = 0;
        double _radius = 0;
        Radians _goal_delta;
};