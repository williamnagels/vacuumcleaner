#include <thread>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vacuumcleaner/cleaningAction.h>
#include "nav_msgs/OccupancyGrid.h"
#include <tf2_ros/transform_listener.h>
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
  uint64_t _map_x = 0;
  uint64_t _map_y = 0;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
public:

  CleaningAction(std::string const& name) :
    _action_server(_node_handle, name,false),
    _action_name(name),
    _tf_listener(_tf_buffer)
  {
    std::string map_topic;
    if (not _node_handle.getParam("map_topic", map_topic))
    {
      ROS_INFO_STREAM("No map topic provided, using default:\"map\"");
      map_topic = "map";
    }

    _map_subscriber = _node_handle.subscribe(map_topic, 1000, &CleaningAction::OnMap, this);
    _action_server.registerGoalCallback([](){ROS_INFO_STREAM("goal callback: "<<std::this_thread::get_id());});
    _action_server.start();
  }

  int8_t GetXY(nav_msgs::OccupancyGrid::ConstPtr const& map, uint64_t x, uint64_t y)
  {
    return map->data[map->info.width * x + y];
  }
  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
  { 
    _map = new_map;
    ROS_INFO_STREAM("OnMap: "<<new_map->info); 
    ROS_INFO_STREAM("GETXY: "<<GetXY(new_map, 0 , 0));
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = _tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0));
      ROS_INFO_STREAM("TRANSFORM: "<< transform); 
      uint64_t grid_x = (uint64_t)((_map_x - _map->info.origin.position.x) / _map->info.resolution);
      uint64_t grid_y = (uint64_t)((_map_y - _map->info.origin.position.y) / _map->info.resolution);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
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
