# Vacuum cleaner robot

* Odometry SLAM
* Lidar
* Differential drive
* NO camera support

## SLAM
SLAM is provided by the gmapping ROS node. The gmapping node is currently configured to require stable odometry. This is provided
by the differential drive implementation:
* Simulation: ros differential drive plugin
* Physical robot: TODO

## ROS simulation
* Run in exploration mode using: roslaunch vacuumcleaner exploration.launch radius:=ROBOT_RADIUS
* Drive around by sending position goals in RVIZ using: roslaunch vacuumcleaner simulation.launch radius:=ROBOT_RADIUS

### launch file parameters
ROBOT_RADIUS: mandatory -> radius of body disk and unit is meters
