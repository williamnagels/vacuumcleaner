#include "map.h"
#include "movement.h"
#include <vacuumcleaner/cleaningAction.h>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

namespace
{
  std::string GetParameter(ros::NodeHandle const& node, std::string const& parameter, std::string const& default_value)
  {
    std::string parameter_temp;    
    if (not node.param(parameter, parameter_temp, default_value))
    {
      ROS_INFO_STREAM("parameter \""<<parameter<<"\" not found, using default:\""<<default_value<<"\"");
    }
    return parameter_temp;
  }
}

class CleaningAction
{
private:
  ros::NodeHandle _node_handle;
  actionlib::SimpleActionServer<vacuumcleaner::cleaningAction> _action_server;
  std::string _action_name;
  vacuumcleaner::cleaningFeedback _feedback;
  const uint64_t RADIUS = 7;
  Map _map;
  Movement _movement;
public:

  CleaningAction(std::string const& name)
    :_action_server(_node_handle, name,false)
    ,_action_name(name)
    ,_map(_node_handle, GetParameter(_node_handle, "map_topic", "map"))
    ,_movement()
  {
    _action_server.registerGoalCallback([](){ROS_INFO_STREAM("goal callback: "<<std::this_thread::get_id());});
    _action_server.start();
  }

  void OnMoveGoalCompletion(const actionlib::SimpleClientGoalState& /*state*/,  const move_base_msgs::MoveBaseResultConstPtr& result) 
  {
    ROS_INFO_STREAM("Result"<< result); 
  }

  void OnGoal(vacuumcleaner::cleaningGoalConstPtr /*goal*/)
  {
    ROS_INFO_STREAM("OnGoal: "<<std::this_thread::get_id()); 
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning");
  CleaningAction cleaningAction("cleaning");
  ros::spin();

  return 0;
}
