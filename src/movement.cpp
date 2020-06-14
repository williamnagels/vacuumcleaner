#include "movement.h"

Movement::Movement()
    : _client("move_base", true)
    , _tf_listener(_tf_buffer)	
{
    while(not _client.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
}

//
void Movement::MoveTo(Coordinates coordinates, OnMovementDone callback)
{
   move_base_msgs::MoveBaseGoal goal;
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();
   goal.target_pose.pose.position.x = coordinates.x();
   goal.target_pose.pose.position.y = coordinates.y();
   goal.target_pose.pose.orientation.w = 1.0;
   _client.sendGoal(goal, callback);
}

Coordinates Movement::GetCurrentPosition()
{
  geometry_msgs::TransformStamped map_transform;
  try
  {
    map_transform = _tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0));
    return { map_transform.transform.translation.x, map_transform.transform.translation.y};
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    return {std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
  }
}