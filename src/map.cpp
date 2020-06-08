#include "map.h"

Map::Map(ros::NodeHandle& _node_handle, std::string const& map_topic)
  :_map_subscriber(_node_handle.subscribe(map_topic, 1000, &Map::OnMap, this))
{
}

void Map::OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
{
  if (_local_map.info.width != new_map->info.width or _local_map.info.height != new_map->info.height)
  {
    ROS_INFO_STREAM("Received uncompatible map. Resetting map.");
    _local_map = *new_map;
  }
  else
  {
    ROS_INFO_STREAM("Updating local map with new data.");

    //Loop over all cells and figure out if the new map has relevant information
    //
    for (uint64_t i = 0; i < _local_map.info.width * _local_map.info.height; i++)
    {
      if (_local_map.data[i] < new_map->data[i])
      {
        _local_map.data[i] = new_map->data[i];
      }
    }
  }
}
  /*
   * x = up
   * y = down
   */
Map::CellState Map::GetState(Map::CellIndex index)
{
  if (index(0) < 0 or index(1) < 0)
  {
    return Map::CellState::Unknown;
  }

  if (index(0) > _local_map.info.height or index(1) > _local_map.info.width)
  {
    return Map::CellState::Unknown;
  }

  //ROS_INFO_STREAM("accessing: [" << (map->info.width * y + x)<<"]");
  return Map::CellState::Free;//_local_map.data[_local_map.info.width * y + x];
}
