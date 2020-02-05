#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vacuumcleaner/cleaningAction.h>


class CleaningAction
{
private:
  ros::NodeHandle _node_handle;
  actionlib::SimpleActionServer<vacuumcleaner::cleaningAction> _action_server; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string _action_name;

  // create messages that are used to published feedback/result
  vacuumcleaner::cleaningFeedback _feedback;
  
public:

  CleaningAction(std::string const& name) :
    _action_server(_node_handle, name, [this](vacuumcleaner::cleaningGoalConstPtr ptr){this->executeCB(ptr);},false),
    _action_name(name)
  {
    _action_server.start();
  }

  ~CleaningAction(void)
  {
  }

  void executeCB(vacuumcleaner::cleaningGoalConstPtr goal)
  {
    ros::Rate r(1);
    bool success = true;

    uint64_t number_of_tiles_visited = 0;
    uint64_t number_of_tiles_to_visit = 100;

    while(number_of_tiles_to_visit)
    {
      number_of_tiles_visited++;
      number_of_tiles_to_visit--;

      // check that preempt has not been requested by the client
      if (_action_server.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", _action_name.c_str());
        // set the action state to preempted
        _action_server.setPreempted();
        success = false;
        break;
      }
      _feedback.number_of_tiles_visited = number_of_tiles_visited;
      _action_server.publishFeedback(_feedback);
      r.sleep();
       
    }

    if(success)
    {
      ROS_INFO("%s: Succeeded", _action_name.c_str());
      vacuumcleaner::cleaningResult result;
      result.number_of_tiles_visited = number_of_tiles_visited;
      _action_server.setSucceeded(result);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleaning");

  CleaningAction cleaningAction("cleaning");
  ros::spin();

  return 0;
}
