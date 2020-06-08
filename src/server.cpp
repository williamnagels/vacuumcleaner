#include "map.h"
#include <thread>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <vacuumcleaner/cleaningAction.h>
#include "nav_msgs/OccupancyGrid.h"
#include <tf2_ros/transform_listener.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
class CleaningAction
{
private:
  ros::NodeHandle _node_handle;
  //vacuumcleaner::cleaningGoalConstPtr _goal; ///< received goal from actionlib client
  tf2_ros::Buffer _tf_buffer;
  actionlib::SimpleActionServer<vacuumcleaner::cleaningAction> _action_server;
  std::string _action_name;
  vacuumcleaner::cleaningFeedback _feedback;
  tf2_ros::TransformListener _tf_listener;
  MoveBaseClient _move_base_client; ///< Will be used to send position goals
  const uint64_t RADIUS = 7;
public:

  CleaningAction(std::string const& name) :
    _action_server(_node_handle, name,false),
    _action_name(name),
    _tf_listener(_tf_buffer),
    _move_base_client("move_base", true)
  {
    std::string map_topic("map");
    if (not _node_handle.getParam("map_topic", map_topic))
    {
      ROS_INFO_STREAM("No map topic provided, using default:\""<<map_topic<<"\"");
    }
    
    while(not _move_base_client.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    _action_server.registerGoalCallback([](){ROS_INFO_STREAM("goal callback: "<<std::this_thread::get_id());});
    _action_server.start();
  }

  /*
   * x = up
   * y = down
   */
//  int8_t GetXY(int64_t x, int64_t y)
//  {
//    if (x < 0 or y < 0)
//    {
//      return -1;
//    }
//
//    if (x > _local_map.info.height or y > _local_map.info.width)
//    {
//      return -1;
//    }
//
//    //ROS_INFO_STREAM("accessing: [" << (map->info.width * y + x)<<"]"); 
//    return _local_map.data[_local_map.info.width * y + x];
//  }

  /*
   * x = up
   * y = down
   */
//  int8_t GetRelative(int64_t x, int64_t y)
//  {
//    // current map tile
//    uint64_t grid_x = (uint64_t)((_map_x - _local_map.info.origin.position.x) / _local_map.info.resolution);
//    uint64_t grid_y = (uint64_t)((_map_y - _local_map.info.origin.position.y) / _local_map.info.resolution);
//     
//    //ROS_INFO_STREAM("current: "<< _map_x<<", "<< _map_y);
//    //ROS_INFO_STREAM("grid: "<< grid_x<<", "<< grid_y);
//    int64_t actual_x = grid_x + x;
//    int64_t actual_y = grid_y + y;
//
//    return GetXY(actual_x, actual_y);
//  }
//  int8_t Get(int64_t x, int64_t y)
//  {
//    // current map tile
//    uint64_t grid_x = (uint64_t)((_map_x - _local_map.info.origin.position.x) / _local_map.info.resolution);
//    uint64_t grid_y = (uint64_t)((_map_y - _local_map.info.origin.position.y) / _local_map.info.resolution);
//    
//    //transform from relative coords to _map coordinates
//    //(0,0) is upper left corner.
//    int64_t actual_x = grid_x + x - RADIUS/2 -1;
//    int64_t actual_y = grid_y + y - RADIUS/2 -1;
//    
//    return GetXY(actual_x, actual_y);
//  }
  void OnMoveGoalCompletion(const actionlib::SimpleClientGoalState& /*state*/,  const move_base_msgs::MoveBaseResultConstPtr& result) 
  {
    ROS_INFO_STREAM("Result"<< result); 
  }

// void SetDirection()
//  {
//    geometry_msgs::TransformStamped map_transform;
//    try
//    {
//      map_transform = _tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0));
//      _map_x = map_transform.transform.translation.x;
//      _map_y = map_transform.transform.translation.y;
//     }
//    catch (tf2::TransformException &ex) 
//    {
//      ROS_WARN("%s",ex.what());
//    }
// 
//
////    ROS_INFO_STREAM("START");
////    ROS_INFO_STREAM("[" << static_cast<int32_t>(GetRelative(-1, 1)) << ","<<static_cast<int32_t>(GetRelative(0, 1))<< ","<<static_cast<int32_t>(GetRelative(1, 1))<<"]"); 
////    ROS_INFO_STREAM("[" << static_cast<int32_t>(GetRelative(-1, 0)) << ","<<static_cast<int32_t>(GetRelative(0, 0))<< ","<<static_cast<int32_t>(GetRelative(1, 0))<<"]"); 
////    ROS_INFO_STREAM("[" << static_cast<int32_t>(GetRelative(-1, -1)) << ","<<static_cast<int32_t>(GetRelative(0, -1))<< ","<<static_cast<int32_t>(GetRelative(1, -1))<<"]");    
////    ROS_INFO_STREAM("END");
//    //ROS_INFO_STREAM("choppa: "<< static_cast<int32_t>(GetRelative(0,0)));
//    //    
//    _direction = GetNewDirection();
//    if (_direction.x == 0 and _direction.y == 0)
//    {
//      ROS_INFO_STREAM("No possible goal in sight. Do intelligent things here.");  
//      return;
//    }
//    float translation_x = _direction.x * _local_map.info.resolution;
//    float translation_y = _direction.y * _local_map.info.resolution;
//    //ROS_INFO_STREAM("Current position in map frame: ("<<_map_x<<","<<_map_y<<")"); 
//    move_base_msgs::MoveBaseGoal goal;
//    goal.target_pose.header.frame_id = "map";
//    goal.target_pose.header.stamp = ros::Time::now();
//    goal.target_pose.pose.position.x = _map_x + translation_x;
//    goal.target_pose.pose.position.y = _map_y + translation_y;
//    goal.target_pose.pose.orientation.w = 1.0;
//    //ROS_INFO_STREAM(goal); 
//    _move_base_client.sendGoal(goal, [this](auto const& a, auto const& b){this->OnMoveGoalCompletion(a, b);});
// 
//  }

//  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
//  { 
//    if (_local_map.info.width != new_map->info.width or _local_map.info.height != new_map->info.height)
//    {
//      ROS_INFO_STREAM("Received uncompatible map. Resetting map.");
//      _local_map = *new_map;
//    }
//    else
//    {
//      ROS_INFO_STREAM("Updating local map with new data.");
//      
//      //Loop over all cells and figure out if the new map has relevant information
//      //
//      for (uint64_t i = 0; i < _local_map.info.width * _local_map.info.height; i++)
//      {
//        if (_local_map.data[i] < new_map->data[i])
//	{
//	  _local_map.data[i] = new_map->data[i];
//	}
//      }
//    }
//  }
//
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
