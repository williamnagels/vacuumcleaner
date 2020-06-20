#include "../types.h"

struct SpiralPlanner
{
    SpiralPlanner(double radius);

    Coordinates GetNewCoordinates();
    
    private:
        double _u = 0;
        double _radius = 0;
};