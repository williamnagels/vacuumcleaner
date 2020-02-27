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
  ros::Timer _timer;  
 
  vacuumcleaner::cleaningGoalConstPtr _goal; ///< received goal from actionlib client
  nav_msgs::OccupancyGrid::ConstPtr _map;
  double _map_x = 0;
  double _map_y = 0;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;

  const uint64_t RADIUS = 7;
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
    _timer = _node_handle.createTimer(ros::Duration(1.0), [this](ros::TimerEvent const& Timer){
		    this->OnTick(Timer);
		    });
    _action_server.registerGoalCallback([](){ROS_INFO_STREAM("goal callback: "<<std::this_thread::get_id());});
    _action_server.start();
  }

  int8_t GetXY(nav_msgs::OccupancyGrid::ConstPtr const& map, int64_t x, int64_t y)
  {
    if (x < 0 or y < 0)
    {
      return -1;
    }

    if (x > map->info.width or y > map->info.height)
    {
      return -1;
    }
     return map->data[map->info.width * x + y];
  }
  int8_t GetRelative(int64_t x, int64_t y)
  {
    // current map tile
    uint64_t grid_x = (uint64_t)((_map_x - _map->info.origin.position.x) / _map->info.resolution);
    uint64_t grid_y = (uint64_t)((_map_y - _map->info.origin.position.y) / _map->info.resolution);
     
    int64_t actual_x = grid_x - x;
    int64_t actual_y = grid_y - y;

    return GetXY(_map, actual_x, actual_y);
  }
  int8_t Get(int64_t x, int64_t y)
  {
    // current map tile
    uint64_t grid_x = (uint64_t)((_map_x - _map->info.origin.position.x) / _map->info.resolution);
    uint64_t grid_y = (uint64_t)((_map_y - _map->info.origin.position.y) / _map->info.resolution);
    
    //transform from relative coords to _map coordinates
    //(0,0) is upper left corner.
    int64_t actual_x = grid_x + x - RADIUS/2 -1;
    int64_t actual_y = grid_y + y - RADIUS/2 -1;
    
    return GetXY(_map, actual_x, actual_y);
  }
  void OnTick(ros::TimerEvent const& Timer)
  {
    ROS_INFO_STREAM("START");
    ROS_INFO_STREAM("[" << static_cast<int32_t>(GetRelative(-1, -1)) << ","<<static_cast<int32_t>(GetRelative(0, -1))<< ","<<static_cast<int32_t>(GetRelative(1, -1))<<"]"); 
    ROS_INFO_STREAM("[" << static_cast<int32_t>(GetRelative(-1, 0)) << ","<<static_cast<int32_t>(GetRelative(0, 0))<< ","<<static_cast<int32_t>(GetRelative(1, 0))<<"]"); 
    ROS_INFO_STREAM("[" << static_cast<int32_t>(GetRelative(-1, 1)) << ","<<static_cast<int32_t>(GetRelative(0, 1))<< ","<<static_cast<int32_t>(GetRelative(1, -1))<<"]"); 
    ROS_INFO_STREAM("END");

  }
  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
  { 
    _map = new_map;
    ROS_INFO_STREAM("OnMap: "<<new_map->info); 
    geometry_msgs::TransformStamped map_transform;
    try
    {
      map_transform = _tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0));
      ROS_INFO_STREAM("TRANSFORM: "<< map_transform); 
      _map_x = map_transform.transform.translation.x;
      _map_y = map_transform.transform.translation.y;
      ROS_INFO_STREAM("map.info.x: "<< _map->info.origin.position.x << " map.info.y: "<< _map->info.origin.position.y); 
      ROS_INFO_STREAM("_map_x: "<< _map_x << " _map_y: "<< _map_y); 
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
