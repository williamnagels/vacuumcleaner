#include "movement.h"
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
void Movement::MoveTo(Pose pose, OnStateChange callback)
{
   move_base_msgs::MoveBaseGoal goal;
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();
   goal.target_pose.pose.position.x = pose.coordinates.x();
   goal.target_pose.pose.position.y = pose.coordinates.y();

  tf2::Quaternion quaternion;
   quaternion.setRPY( 0, 0, pose.angle);
   quaternion.normalize();
   tf2::convert(quaternion, goal.target_pose.pose.orientation);

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