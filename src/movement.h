#pragma once

#include "types.h"
#include "pose_generator.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using PositionChangedCallback = std::function<void(Coordinates)>;
class Movement
{
public:
    Movement(PoseGenerator& generator, PositionChangedCallback position_changed_callback);

    void FeedbackCallback(move_base_msgs::MoveBaseFeedbackConstPtr const& feedback);
    Coordinates GetCurrentPosition();
private:
    void MoveTo(Pose const& pose);
    MoveBaseClient _client;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener; ///< Used to figure out robot position in map frame
    move_base_msgs::MoveBaseGoal _active_goal;
    double _distance_before_scheduling_new_goal;
    double _distance_before_position_changed_callback;
    PoseGenerator& _generator;
    Eigen::Vector2d _last_known_position;
    PositionChangedCallback _position_changed_callback;
};