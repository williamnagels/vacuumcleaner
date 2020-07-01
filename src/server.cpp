
#include "map.h"
#include "movement.h"
#include "parameter.h"

#include "route-planning/spiral_planner.h"

#include <vacuumcleaner/cleaningAction.h>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

class CleaningAction
{
private:
  ros::NodeHandle _node_handle;
  actionlib::SimpleActionServer<vacuumcleaner::cleaningAction> _action_server;
  std::string _action_name;
  vacuumcleaner::cleaningFeedback _feedback;
  Map _map;
  SpiralPlanner _spiral_planner;
  Movement _movement;
public:

  CleaningAction(std::string const& name)
    :_action_server(_node_handle, name,false)
    ,_action_name(name)
    ,_map(_node_handle)
    ,_movement(_spiral_planner, [this](auto coordinates){_map.OnPositionChanged(coordinates);})
  {
    _action_server.registerGoalCallback(std::bind(std::mem_fn(&CleaningAction::OnGoal), this));
    _action_server.start();
  }

  void OnGoal()
  {

  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning");
  CleaningAction cleaningAction("cleaning");
  ros::spin();

  return 0;
}
