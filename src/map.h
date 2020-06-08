#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Dense>
class Map
{
public:
  Map(ros::NodeHandle& handle, std::string const& map_topic); ///< Create Map that will subscribe some topic to get occupancy grid updates from.
  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map); ///< Callback for occupancy grid updates

  enum class CellState
  {
    Free, 
    Visited,
    Blocked,
    Unknown,
  };

  using CellIndex = Eigen::Vector2d;
  CellState GetState(CellIndex);
private:
  Eigen::Vector2d _position;
  nav_msgs::OccupancyGrid _local_map; ///< Map from SLAM node
  ros::Subscriber _map_subscriber; ///< Will subscribe to map updates
};
