#include "movement.h"
 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


Movement::Movement(PoseGenerator& generator, PositionChangedCallback position_changed_callback)
    : _client("move_base", true)
    , _distance_before_scheduling_new_goal(0.1)
    , _distance_before_position_changed_callback(0.02)
    , _generator(generator)
    , _position_changed_callback(position_changed_callback)
{
    while(not _client.waitForServer())
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    MoveTo(_generator.Generate());
}

void  Movement::DoneCallback(actionlib::SimpleClientGoalState const& state, move_base_msgs::MoveBaseResultConstPtr const& result)
{
  ROS_INFO_STREAM("state: "<<state.toString());
  ROS_INFO_STREAM("result: "<<*result);
}

void  Movement::ActiveCallback()
{
  ROS_INFO_STREAM("active?: ");
}

void Movement::FeedbackCallback(move_base_msgs::MoveBaseFeedbackConstPtr const& feedback)
{
  Eigen::Vector2d v1{feedback->base_position.pose.position.x, feedback->base_position.pose.position.y};
  Eigen::Vector2d v2{_active_goal.target_pose.pose.position.x, _active_goal.target_pose.pose.position.y};

  double remaining_distance_to_travel = std::sqrt((v1 - v2).array().square().sum());

  ROS_DEBUG_STREAM("current position: "<<feedback->base_position.pose.position << " active goal: "<< _active_goal.target_pose.pose.position<< " distance: "<<remaining_distance_to_travel);
  if (remaining_distance_to_travel < _distance_before_scheduling_new_goal)
  {
   MoveTo(_generator.Generate());
  } 

  double distance_traveled_since_known_position = std::sqrt((v1 - _last_known_position).array().square().sum());
  
  if (distance_traveled_since_known_position > _distance_before_position_changed_callback)
  {
    _last_known_position = v1;
    _position_changed_callback(_last_known_position);
  } 
}

void Movement::MoveTo(Pose const& pose)
{
  _active_goal.target_pose.header.frame_id = "map";
  _active_goal.target_pose.header.stamp = ros::Time::now();
  _active_goal.target_pose.pose.position.x = pose.coordinates.x();
  _active_goal.target_pose.pose.position.y = pose.coordinates.y();

  tf2::Quaternion quaternion;
   quaternion.setRPY( 0, 0, pose.angle);
   quaternion.normalize();
   tf2::convert(quaternion, _active_goal.target_pose.pose.orientation);

  _client.sendGoal(
    _active_goal, 
    std::bind(&Movement::DoneCallback, this, std::placeholders::_1, std::placeholders::_2), 
    std::bind(&Movement::ActiveCallback, this),
    std::bind(&Movement::FeedbackCallback, this, std::placeholders::_1)
    );
}