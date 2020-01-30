# Vacuum cleaner robot
Variable contour autonomous differential drive vacuum cleaner

## ROS simulation
- Run in exploration mode using: roslaunch vacuumcleaner exploration.launch radius:=ROBOT_RADIUS
- Drive around by sending position goals in RVIZ using: roslaunch vacuumcleaner simulation.launch radius:=ROBOT_RADIUS

### launch file parameters
ROBOT_RADIUS: mandatory -> radius of body disk and unit is meters
