# Vacuum cleaner robot

## Brief
Scan data of a LIDAR is combined with odometry provided by the differential drive to implement a SLAM capable platform.
The resulting map is used by an online coverage algorithm to plan a route.

## How to build
## How to run
* Run in exploration mode using: roslaunch vacuumcleaner exploration.launch radius:=ROBOT_RADIUS
* Drive around by sending position goals in RVIZ using: roslaunch vacuumcleaner simulation.launch radius:=ROBOT_RADIUS

### launch file parameters
ROBOT_RADIUS: mandatory -> radius of body disk and unit is meters
