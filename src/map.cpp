#include "map.h"
#include "parameter.h"
Map::Map(ros::NodeHandle& _node_handle)
  :_map_subscriber(_node_handle.subscribe(GetParameter(PARAM_VISITED_MAP_SUBSCRIBE_TOPIC, std::string("map")), 1000, &Map::OnMap, this))
  ,_map_publisher(_node_handle.advertise<nav_msgs::OccupancyGrid>(GetParameter(PARAM_VISITED_MAP_PUBLISHED_TOPIC, std::string("updated_map")), 1000))
{
}

void Map::OnMap(nav_msgs::OccupancyGrid::Ptr const& new_map)
{
  GridDimensionType amount_of_cells = new_map->info.width * new_map->info.height;

  if (not _map or _map->info.width * _map->info.height != amount_of_cells)
  {
    _map = new_map;
    return;
  }
  
  // std::transform(
  //   std::begin(new_map->data), std::end(new_map->data), // Input 1
  //   std::begin(_map->data), // Input 2
  //   std::begin(_map->data), //Output
  //   [this](GridValueType n, GridValueType o){return this->Convert(n, o);});

}
void Map::OnPositionChanged(Coordinates position)
{
  if (not _map) return;

  CellIndex cell_index = Get(position);
  Set(cell_index, CellState::Visited);
  
  _map_publisher.publish(_map);
}
auto Map::Get(Coordinates position) -> CellIndex
{
  Coordinates offset_corrected = position - Coordinates{_map->info.origin.position.x, _map->info.origin.position.y};  

  return {offset_corrected.x() /  _map->info.resolution, offset_corrected.y() /  _map->info.resolution};
}


auto Map::ToArrayIndex(CellIndex cell_index) const -> GridDimensionType
{
  cell_index.y() *= (_map->info.width * _map->info.resolution);
  return cell_index.sum();
}

auto Map::Get(Map::CellIndex cell_index) -> CellState 
{
  return Convert(_map->data[ToArrayIndex(cell_index)]);
}
void Map::Set(CellIndex cell_index, CellState new_state)
{
  _map->data[ToArrayIndex(cell_index)] = Convert(new_state);
}
auto Map::Convert(GridValueType /*Value*/) const -> CellState
{
  /*if(Value == -1)
  {
    return CellState::Unknown;
  }
  // 0 - 100 means not blocked; 100 means blocked
  else if(Value < _free_blocked_threshold)
  {
    return CellState::Free;
  }
  else if (Value ==  _visited_threshold)
  {
    return CellState::Visited;
  }
  else
  {
    return CellState::Blocked;
  }*/
  return CellState::Visited;
}

auto Map::Convert(Map::CellState value) const -> GridValueType 
{
  if(value == CellState::Unknown)
  {
    return -1;
  }
  // 0 - 100 means not blocked; 100 means blocked
  else if(value == CellState::Free)
  {
    return _free_blocked_threshold - 1;
  }
  else if(value == CellState::Visited)
  {
    return _visited_threshold;
  }
  else
  {
    return _free_blocked_threshold;
  }
}

auto Map::Convert(GridValueType new_state_value, GridValueType /*existing_state*/) -> GridValueType 
{
  return new_state_value;
}