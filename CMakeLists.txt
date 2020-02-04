cmake_minimum_required(VERSION 2.8.3)
project(vacuumcleaner)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
	FILES vacuumcleaner.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(vacuumcleaner_server src/vacuumcleaner_server.cpp)

target_link_libraries(
  vacuumcleaner_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  vacuumcleaner_server
  ${vacuumcleaner_EXPORTED_TARGETS}
)
#############
## Install ##
#############


# install roslaunch files
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(TARGETS vacuumcleaner_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############



