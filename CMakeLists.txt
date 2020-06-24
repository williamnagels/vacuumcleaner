cmake_minimum_required(VERSION 3.10)
project(vacuumcleaner)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -pipe -fasynchronous-unwind-tables -fstack-protector -fstack-protector-strong -Wall -Wextra -Werror -Wuninitialized")
endif()
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(
  catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  tf2
  tf
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
	FILES cleaning.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  
  )
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(client src/client.cpp )
target_include_directories(client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

set_property(TARGET client PROPERTY CXX_STANDARD 17)
target_link_libraries( 
  client
  ${catkin_LIBRARIES}
)

add_dependencies(
  client
  ${vacuumcleaner_EXPORTED_TARGETS}
)

add_executable(server src/server.cpp src/map.cpp src/movement.cpp src/route-planning/spiral_planner.cpp)
target_include_directories(server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set_property(TARGET server PROPERTY CXX_STANDARD 17)

target_link_libraries(
  server
  ${catkin_LIBRARIES}
)

add_dependencies(
  server
  ${vacuumcleaner_EXPORTED_TARGETS}
)
#############
## Install ##
#############

# install roslaunch files
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(TARGETS client server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
