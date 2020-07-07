#pragma once
#include "types.h"
#include "ros/param.h"
#include <optional>

template <typename T>
auto GetParameter(std::string const& parameter_name, T const& default_value) -> T
{
    T parameter_temp;    
    
    if (not ros::param::get(parameter_name, parameter_temp))
    {
        ROS_INFO_STREAM("parameter \""<<parameter_name<<"\" not found, using default:\""<<default_value<<"\"");
    }
    else
    {
        ROS_INFO_STREAM("parameter \""<<parameter_name<<"\" found, using:\""<<parameter_temp<<"\"");
    }

     

    return parameter_temp;
}

const std::string PARAM_ROBOT_RADIUS = "/vacuumcleaner/robot/radius";
const double PARAM_DEFAULT_ROBOT_RADIUS = 0.1;
const std::string PARAM_VISITED_MAP_SUBSCRIBE_TOPIC = "/vacuumcleaner/robot/map_subscribe_topic";
const std::string PARAM_VISITED_MAP_PUBLISHED_TOPIC = "/vacuumcleaner/robot/visited_map_publish_topic";
const std::string PARAM_SPIRAL_DELTA_DENOMINATOR = "/vacuumcleaner/algorithm/spiral_delta_denominator";
const Radians PARAM_DEFAULT_SPIRAL_DELTA_DENOMINATOR = 9;