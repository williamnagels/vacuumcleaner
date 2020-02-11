#include <future>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vacuumcleaner/cleaningAction.h>
#include "nav_msgs/OccupancyGrid.h"
class CleaningAction
{
private:
  ros::NodeHandle _node_handle;
  actionlib::SimpleActionServer<vacuumcleaner::cleaningAction> _action_server; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string _action_name;

  // create messages that are used to published feedback/result
  vacuumcleaner::cleaningFeedback _feedback;
  ros::Subscriber _map_subscriber;  

  nav_msgs::OccupancyGrid::ConstPtr _map;
  vacuumcleaner::cleaningGoalConstPtr _goal;

  bool _handled_map_update = true;
  std::promise<nav_msgs::OccupancyGrid::ConstPtr> _map_promise;
  bool PlanRoute()
  {
    ROS_INFO_STREAM("Launching mission");
    ROS_INFO_STREAM("map header: "<< _map->info);
    return false;
  }
public:

  CleaningAction(std::string const& name) :
    _action_server(_node_handle, name, [this](vacuumcleaner::cleaningGoalConstPtr ptr){this->OnGoal(ptr);},false),
    _action_name(name)
  {
    std::string map_topic;
    if (not _node_handle.getParam("map_topic", map_topic))
    {
      ROS_INFO_STREAM("No map topic provided, using default:\"map\"");
      map_topic = "map";
    }

    _map_subscriber = _node_handle.subscribe(map_topic, 1000, &CleaningAction::OnMap, this);
    _action_server.start();
  }

  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& map)
  { 
    ROS_INFO("%s: OnMap callback", _action_name.c_str());
    _map_promise.set_value(map);
  }

  bool PlanRoute(nav_msgs::OccupancyGrid::ConstPtr map)
  {
//   _map_promise = std::promise<nav_msgs::OccupancyGrid::ConstPtr>();
//    ROS_INFO("%s: Planning route", _action_name.c_str());
//    vacuumcleaner::cleaningResult result;
//    result.number_of_tiles_visited = 10;
//    _action_server.setSucceeded(result);
    return true;
  }
  // This runs on simple actionlib thread
  void OnGoal(vacuumcleaner::cleaningGoalConstPtr goal)
  {
    _goal = goal;
    do {} while(true);//PlanRoute(_map_promise.get_future().get()));
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning");
  CleaningAction cleaningAction("cleaning");
  ros::spin();

  return 0;
}
