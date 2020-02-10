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

  enum class State
  {
    WaitingForMap,
    WaitingForGoal,
    Navigating
  };

  void PlanNewRouteIfRequired()
  {
    if(true)
    {
      PlanRoute();
    }
  }
  void PlanRoute()
  {
    ROS_INFO_STREAM("Launching mission");
    ROS_INFO_STREAM("map header: "<< _map->info);
  }
  void GotMap()
  {
    if (_state == State::WaitingForMap)
    {
    	if (not _goal)
	{
	  _state = State::WaitingForGoal;
	}
	else
	{
	  PlanRoute();
	}
    }
    else if (_state == State::Navigating)
    {
      PlanNewRouteIfRequired();
    }
  }
  void GotGoal()
  {
    if (_state == State::WaitingForGoal)
    {
      PlanRoute();
    }
    else    
    {
      //errr? send help
    }
 
  }
  State _state = State::WaitingForMap;
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

  ~CleaningAction(void)
  {
  }

  void TryLaunch()
  {
    if (not _map or not _goal)
    {
      return;   
    }
  }
  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& map)
  {
    _map = map;
    GotMap();
  }
  void OnGoal(vacuumcleaner::cleaningGoalConstPtr goal)
  {
    _goal = goal;
    GotGoal();
//    ros::Rate r(1);
//    bool success = true;
//
//    uint64_t number_of_tiles_visited = 0;
//    uint64_t number_of_tiles_to_visit = 100;
//    ROS_INFO_STREAM("Received goal: " << *goal);
//
//
//    while(number_of_tiles_to_visit)
//    {
//      number_of_tiles_visited++;
//      number_of_tiles_to_visit--;
//
//      // check that preempt has not been requested by the client
//      if (_action_server.isPreemptRequested() || !ros::ok())
//      {
//        ROS_INFO("%s: Preempted", _action_name.c_str());
//        // set the action state to preempted
//        _action_server.setPreempted();
//        success = false;
//        break;
//      }
//      _feedback.number_of_tiles_visited = number_of_tiles_visited;
//      _feedback.number_of_tiles_to_visit = number_of_tiles_to_visit;
//      _action_server.publishFeedback(_feedback);
//      r.sleep();
//       
//    }
//
//    if(success)
//    {
//      ROS_INFO("%s: Succeeded", _action_name.c_str());
//      vacuumcleaner::cleaningResult result;
//      result.number_of_tiles_visited = number_of_tiles_visited;
//      _action_server.setSucceeded(result);
//    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning");
  CleaningAction cleaningAction("cleaning");
  ros::spin();

  return 0;
}
