#include "map.h"
#include "parameter.h"
Map::Map(ros::NodeHandle& _node_handle)
  :_map_subscriber(_node_handle.subscribe(GetParameter(PARAM_VISITED_MAP_SUBSCRIBE_TOPIC, std::string("map")), 1000, &Map::OnMap, this))
  ,_map_publisher(_node_handle.advertise<nav_msgs::OccupancyGrid>(GetParameter(PARAM_VISITED_MAP_PUBLISHED_TOPIC, std::string("updated_map")), 1000))
  ,_robot_radius(GetParameter(PARAM_ROBOT_RADIUS, PARAM_DEFAULT_ROBOT_RADIUS))
{
   UpdateVisited({0, 0});
}

void Map::OnMap(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
{
  GridDimensionType amount_of_cells = new_map->info.width * new_map->info.height;

  if (_map.info.width * _map.info.height != amount_of_cells)
  {
    _map = *new_map;
  }
  
  std::transform(
    std::begin(new_map->data), std::end(new_map->data),
    std::begin(_map.data),
    std::begin(_map.data),
    [this](GridValueType n, GridValueType o){return Convert(n, o);});

}
void Map::UpdateVisited(CellIndex current_cell_index)
{
  uint64_t robot_radius_cells = std::ceil(_robot_radius / _map.info.resolution);
  
  for (int64_t y = -robot_radius_cells; y< (int64_t)robot_radius_cells; y++)
  {
    for (int64_t x = -(robot_radius_cells/*-y*/); x< (int64_t)(robot_radius_cells/*-y*/); x++)
    {
      Set(current_cell_index + CellIndex{x, y}, CellState::Visited);
    }
  }

}
void Map::OnPositionChanged(Coordinates position)
{
  if (not _map.info.height) return;

  CellIndex cell_index = Get(position);
  UpdateVisited(cell_index);

  _map_publisher.publish(_map);
}
auto Map::Get(Coordinates position) -> CellIndex
{
  Coordinates offset_corrected = position - Coordinates{_map.info.origin.position.x, _map.info.origin.position.y};  

  return {offset_corrected.x() /  _map.info.resolution, offset_corrected.y() /  _map.info.resolution};
}


auto Map::ToArrayIndex(CellIndex cell_index) const -> GridDimensionType
{
  cell_index.y() *= _map.info.width;
  return cell_index.sum();
}

auto Map::Get(Map::CellIndex cell_index) -> CellState 
{
  return Convert(_map.data[ToArrayIndex(cell_index)]);
}
void Map::Set(CellIndex cell_index, CellState new_state)
{
  _map.data[ToArrayIndex(cell_index)] = Convert(new_state);
}
auto Map::Convert(GridValueType Value) const -> CellState
{
  if(Value == _unknown_cell_value)
  {
    return CellState::Unknown;
  }
  // 0 - 100 means not blocked; 100 means blocked
  else if(Value < _free_blocked_threshold_cell_value)
  {
    return CellState::Free;
  }
  else if (Value ==  _visited_threshold_cell_value)
  {
    return CellState::Visited;
  }
  else
  {
    return CellState::Blocked;
  }
}

auto Map::Convert(Map::CellState value) const -> GridValueType 
{
  if(value == CellState::Unknown)
  {
    return _unknown_cell_value;
  }
  else if(value == CellState::Free)
  {
    return _free_blocked_threshold_cell_value - 1;
  }
  else if(value == CellState::Visited)
  {
    return _visited_threshold_cell_value;
  }
  else
  {
    return _free_blocked_threshold_cell_value;
  }
}

auto Map::Convert(GridValueType new_state_value, GridValueType existing_state_value) -> GridValueType 
{
  CellState new_state = Convert(new_state_value);
  CellState existing_state = Convert(existing_state_value);

  
  if (new_state == CellState::Blocked or new_state == CellState::Unknown)
  {
    return Convert(new_state);
  }
  else if (new_state == CellState::Free and (existing_state == CellState::Blocked or existing_state == CellState::Unknown))
  {
    return Convert(CellState::Free);
  }

  return Convert(existing_state);

}