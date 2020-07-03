#pragma once
#include "types.h"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Dense>
#include <ros/node_handle.h>
class Map
{
public:
  Map(ros::NodeHandle& handle); ///< Create Map that will subscribe some topic to get occupancy grid updates from.
  void OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map); ///< Callback for occupancy grid updates
  void OnPositionChanged(Coordinates new_position); ///< callback if robot position changed a significantly enough distance
  enum class CellState  
  {
    Free,
    Visited,
    Blocked,
    Unknown,
  };
  
  using GridDimensionType = nav_msgs::OccupancyGrid::_info_type::_width_type;
  using GridValueType = nav_msgs::OccupancyGrid::_data_type::value_type;
  using CellIndex = Eigen::Matrix<GridDimensionType, 2,1>;
  using Dimensions = Eigen::Matrix<GridDimensionType, 2,1>;

  CellState Get(CellIndex); ///< get state of cell at index
  CellIndex Get(Coordinates); ///< convert world coordinates to a cell index
  void Set(CellIndex cell_index, CellState new_state);
  GridValueType Convert(GridValueType new_value, GridValueType old_value);

private:
  CellState Convert(GridValueType) const;
  GridValueType Convert(Map::CellState Value) const;
  GridDimensionType ToArrayIndex(CellIndex cell_index) const;
  nav_msgs::OccupancyGrid _map;
  ros::Subscriber _map_subscriber; ///< Will subscribe to map updates
  ros::Publisher  _map_publisher; ///< Publish updated map
  GridValueType _free_blocked_threshold_cell_value = 70;
  GridValueType _visited_threshold_cell_value = 101;
  GridValueType _unknown_cell_value = -1;
};
