#include "nav_msgs/OccupancyGrid.h"
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

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
  static CellState Convert(int);
private:
  struct LocalMap
  {
    void Update(nav_msgs::OccupancyGrid::ConstPtr const& new_map);
    CellState GetCellState(CellIndex);
    static CellState Convert(int, CellState);
    private:
      std::vector<CellState> _state;
      Dimensions _dimensions;
      void Reset(nav_msgs::OccupancyGrid::ConstPtr const& new_map);
  };
  LocalMap _map;
  Eigen::Vector2d _position;
  void UpdatePosition(); ///< request current robot position from tf2
  ros::Subscriber _map_subscriber; ///< Will subscribe to map updates
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener; ///< Used to figure out robot position in map frame
};
