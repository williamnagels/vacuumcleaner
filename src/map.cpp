#include "map.h"

Map::Map(ros::NodeHandle& _node_handle, std::string const& map_topic)
  :_map_subscriber(_node_handle.subscribe(map_topic, 1000, &Map::OnMap, this))
{
}

Map::CellState Map::Convert(int8_t Value)
{
  if(Value == -1)
  {
    return CellState::Unknown;
  }
  else if(Value < 10)
  {
    return CellState::Free;
  }
  else
  {
    return CellState::Blocked;
  }
}
Map::CellState Map::LocalMap::Convert(int8_t new_state_value, Map::CellState existing_state)
{
  CellState new_state = Map::Convert(new_state_value);
  
  if(existing_state < new_state)
  {
     return new_state;
  }
  else if ( (existing_state == CellState::Blocked or existing_state == CellState::Unknown) and new_state == CellState::Free)
  {
    return CellState::Free;
  }
  else
  {
    return existing_state;
  }
}


void Map::LocalMap::Reset(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
{
  uint64_t amount_of_cells = new_map->info.width * new_map->info.height;
  ROS_INFO_STREAM("Received uncompatible map. Resetting map. New dimensions ("<<new_map->info.width<<","<<new_map->info.height<<") old dimensions ("<<_dimensions.x()<<","<<_dimensions.y()<<")");
  _dimensions.x() = new_map->info.width;
  _dimensions.y() = new_map->info.height;
  _state.resize(amount_of_cells);
  std::transform(std::begin(new_map->data), std::end(new_map->data),std::begin(_state), &Map::Convert);
}
void Map::LocalMap::Update(nav_msgs::OccupancyGrid::ConstPtr const& new_map)
{
  uint64_t amount_of_cells = new_map->info.width * new_map->info.height;

  if (_state.size() != amount_of_cells)
  {
    return Reset(new_map);
  }
  
  std::transform(
    std::begin(new_map->data), std::end(new_map->data), 
    std::begin(_state), std::begin(_state), 
    &Map::LocalMap::Convert);
}

void Map::OnMap(nav_msgs::OccupancyGrid::Ptr const& new_map)
{
  _offset = {new_map->info.origin.position.x, new_map->info.origin.position.y};
  _map.Update(new_map);
}
void Map::OnPositionChanged(Coordinates position)
{
 GetIndex(position);
}
auto Map::GetIndex(Coordinates position) -> CellIndex
{
  CellIndex cell_index;

  Coordinates offset_corrected = position + _offset;
  cell_index.x() = offset_corrected.x() / _map.GetDimensions().x();
  cell_index.y() = offset_corrected.y() / _map.GetDimensions().y();

  return cell_index;
}
auto Map::LocalMap::GetDimensions() const -> Dimensions
{
  return _dimensions;
}
Map::CellState Map::LocalMap::GetCellState(Map::CellIndex cell_index)
{
  if (_dimensions.y() < cell_index.y() or _dimensions.x() < cell_index.x())
  {
    return Map::CellState::Unknown;
  }

  //ROS_INFO_STREAM("accessing: [" << (map->info.width * y + x)<<"]");
  return _state[cell_index.prod()];
}
  /*
   * x = up
   * y = down
   */
Map::CellState Map::GetState(Map::CellIndex cell_index)
{
  return _map.GetCellState(cell_index);
}
