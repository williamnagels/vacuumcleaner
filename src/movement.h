#include "types.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class Movement
{
public:
    Movement();

    using OnStateChange = std::function< void(const actionlib::SimpleClientGoalState& /*state*/,  const move_base_msgs::MoveBaseResultConstPtr& result)>;
    void MoveTo(Coordinates coordinates, OnStateChange callback);
    Coordinates GetCurrentPosition();
private:
    MoveBaseClient _client;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener; ///< Used to figure out robot position in map frame
};