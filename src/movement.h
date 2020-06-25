#include "types.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <queue>
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class Movement
{
public:
    Movement();

    void Add(std::queue<Pose> const& poses);

    void FeedbackCallback(move_base_msgs::MoveBaseFeedbackConstPtr const& feedback);

    Coordinates GetCurrentPosition();
private:

    void MoveTo(Pose const& pose);
    MoveBaseClient _client;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener; ///< Used to figure out robot position in map frame
    move_base_msgs::MoveBaseGoal _active_goal;
    std::queue<Pose> _poses;
    double _distance_before_scheduling_new_goal;
};