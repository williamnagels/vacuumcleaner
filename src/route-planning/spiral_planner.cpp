#include "spiral_planner.h"
#include <math.h>
SpiralPlanner::SpiralPlanner(double radius)
    :_radius(radius)
{

}
Coordinates SpiralPlanner::GetNewCoordinates()
{
    auto UpdatedCoords = Coordinates{
        (_radius / M_PI) * _u * cos(-_u),
        (_radius / M_PI) * _u * sin(-_u)
    };

    _u += M_PI_2;
    

    return UpdatedCoords;
}