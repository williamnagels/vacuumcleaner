#include "types.h"

struct SpiralPlanner
{
    SpiralPlanner(double radius, Radians goal_delta);

    Pose GetNewPose();
    
    private:
        Radians _u = 0;
        double _radius = 0;
        Radians _goal_delta;
        Coordinates Calculate(double u) const;
};