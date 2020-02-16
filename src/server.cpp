#include <thread>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vacuumcleaner/cleaningAction.h>
#include "nav_msgs/OccupancyGrid.h"
class CleaningAction
{
private:
  ros::NodeHandle _node_handle;
  actionlib::SimpleActionServer<vacuumcleaner::cleaningAction> _action_server;
  std::string _action_name;
  vacuumcleaner::cleaningFeedback _feedback;
  ros::Subscriber _map_subscriber; ///< Will subscribe to gmappping map updates.
  vacuumcleaner::cleaningGoalConstPtr _goal; ///< received goal from actionlib client
  nav_msgs::OccupancyGrid::ConstPtr _map;  
public:

  CleaningAction(std::string const& name) :
    _action_server(_node_handle, name,false),
    _action_name(name)
  {
    std::string map_topic;
    if (not _node_handle.getParam("map_topic", map_topic))
    {
      ROS_INFO_STREAM("No map topic provided, using default:\"map\"");
      map_topic = "map";
    }

    _map_subscriber = _node_handle.subscribe(map_topic, 1000, &CleaningAction::OnMap, this);
    _action_server.registerGoalCallback([](){ROS_INFO_STREAM("goal callback: "<<std::this_thread::get_id());});
    //_action_server.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

    _action_server.start();
  }

  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
  { 
    ROS_INFO_STREAM("OnMap: "<<std::this_thread::get_id()); 
    _map = new_map;
  }

  void OnGoal(vacuumcleaner::cleaningGoalConstPtr goal)
  {
    ROS_INFO_STREAM("OnGoal: "<<std::this_thread::get_id()); 
    _goal = _action_server.acceptNewGoal();
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning");
  CleaningAction cleaningAction("cleaning");
  ros::spin();

  return 0;
}
