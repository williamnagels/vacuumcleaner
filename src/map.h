#pragma once

#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Dense>
#include <ros/node_handle.h>
class Map
{
public:
  Map(ros::NodeHandle& handle, std::string const& map_topic); ///< Create Map that will subscribe some topic to get occupancy grid updates from.
  void OnMap(nav_msgs::OccupancyGrid::Ptr const& new_map); ///< Callback for occupancy grid updates

  enum class CellState  
  {
    Free=0,
    Visited=1,
    Blocked=2,
    Unknown=3,
  };
  using CellIndex = Eigen::Matrix<uint64_t, 2,1>;
  using Dimensions = Eigen::Matrix<uint64_t, 2,1>;

  CellState GetState(CellIndex);
  static CellState Convert(int8_t);
private:
  struct LocalMap
  {
    void Update(nav_msgs::OccupancyGrid::ConstPtr const& new_map);
    CellState GetCellState(CellIndex);
    static CellState Convert(int8_t, CellState);
    private:
      std::vector<CellState> _state;
      Dimensions _dimensions;
      void Reset(nav_msgs::OccupancyGrid::ConstPtr const& new_map);
  };
  LocalMap _map;

  ros::Subscriber _map_subscriber; ///< Will subscribe to map updates

};
