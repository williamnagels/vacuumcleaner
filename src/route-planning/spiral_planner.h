#include "../types.h"

struct SpiralPlanner
{
    SpiralPlanner(double radius);

    Pose GetNewPose();
    
    private:
        double _u = 0;
        double _radius = 0;

        Coordinates Calculate(double u) const;
};