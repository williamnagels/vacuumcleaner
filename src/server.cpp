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
  vacuumcleaner::cleaningResult _result;
  
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
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    _feedback.sequence.clear();
    _feedback.sequence.push_back(0);
    _feedback.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", _action_name.c_str(), goal->order, _feedback.sequence[0], _feedback.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (_action_server.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", _action_name.c_str());
        // set the action state to preempted
        _action_server.setPreempted();
        success = false;
        break;
      }
      _feedback.sequence.push_back(_feedback.sequence[i] + _feedback.sequence[i-1]);
      _action_server.publishFeedback(_feedback);
      
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      ROS_INFO("%s: Succeeded", _action_name.c_str());
      _result.sequence = _feedback.sequence;
      _action_server.setSucceeded(_result);
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
