#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vacuumcleaner/cleaningAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "cleaning");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<vacuumcleaner::cleaningAction> _action_client("cleaning", true);

  ROS_INFO("Waiting for action server to start.");
  _action_client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  vacuumcleaner::cleaningGoal goal;
  goal.zone = 0;
  _action_client.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = _action_client.waitForResult(ros::Duration(20.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = _action_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {

    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}
