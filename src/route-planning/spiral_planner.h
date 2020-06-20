#include "../types.h"

struct SpiralPlanner
{
    SpiralPlanner(uint64_t radius);

    Coordinates GetNewCoordinates();
    
    private:
        double _u = 0;
        uint64_t _radius = 0;
};