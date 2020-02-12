#include <mutex>
#include <condition_variable>
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

  struct ThreadSafeOccupancyGrid
  {
  private:
    nav_msgs::OccupancyGrid::ConstPtr _map;  
    std::mutex _mutex; 
  public:

   void ActionThreadSafe(std::function<void(nav_msgs::OccupancyGrid::ConstPtr)> Action)
   {
     std::lock_guard<std::mutex> lock(_mutex);
     if (not _map)
     { 
       return;
     }

     return Action(_map);
   } 
   void UpdateThreadSafe(nav_msgs::OccupancyGrid::ConstPtr map_to_set) 
   { 
     std::lock_guard<std::mutex> lock(_mutex);
     _map = map_to_set;
     ROS_INFO_STREAM("Updating map threadsafe");
   } 
  };

  ThreadSafeOccupancyGrid _map; ///< Actionlibserver runs in a different thread than callbacks are called on
  std::condition_variable _new_map_notification; ///< called whenever subscriber receives a new map from gmapping. Will wake action server thread.
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

  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
  { 
    _map.UpdateThreadSafe(new_map);
    _new_map_notification.notify_one();
  }

 // This runs on simple actionlib thread
  void OnGoal(vacuumcleaner::cleaningGoalConstPtr goal)
  {
    _goal = goal;
    std::mutex signal_mutex;
    std::unique_lock<std::mutex> lock(signal_mutex);
    while(true)
    {
       _new_map_notification.wait(lock);
       _map.ActionThreadSafe([](nav_msgs::OccupancyGrid::ConstPtr Map){ROS_INFO_STREAM("doing threadsafe action");});
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning");
  CleaningAction cleaningAction("cleaning");
  ros::spin();

  return 0;
}
