#pragma once
#include "types.h"
struct PoseGenerator
{
    virtual Pose Generate() = 0;
};