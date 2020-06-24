# Vacuum cleaner robot

  

## Brief

Scan data of a LIDAR is combined with odometry provided by the differential drive to implement a SLAM capable platform.

The resulting map is used by an online coverage algorithm to plan a route.

  

## How to build

## How to run

* Start default planning algorithm in default room: roslaunch vacuumcleaner simulation.launch robot_radius:=X spiral_delta_denominator:=Y

  

### launch file parameters

robot_radius: mandatory -> radius of body disk and unit is meters

spiral_delta_denominator: mandatory -> cfr planning algorithm. Subsequent goal points for stage 1 spiral planning lay 
<img src="https://render.githubusercontent.com/render/math?math=\dfrac{\pi}{\text{spiral_delta_denominator}}" /> radians apart.