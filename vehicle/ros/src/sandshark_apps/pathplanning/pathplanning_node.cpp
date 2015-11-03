#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>

#include <ros/ros.h>
#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/CurrentObjective.h>
#include <sandshark_msgs/MavlinkObjectiveCommand.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class PathPlanningApp: public TaskBase {
  private:
    ros::Subscriber _navSub;
    ros::Subscriber _objSub;

    ros::Publisher _cmdPub;
    sandshark_msgs::MavlinkObjectiveCommand _cmdMsg;

    struct CurrentObjective {
        double prev_end_time;
        int prev_coord_frame;
        int prev_command;
        int prev_command_id;
        double prev_param1;
        double prev_param2;
        double prev_param3;
        double prev_param4;
        double prev_param5;
        double prev_param6;
        double prev_param7;
        bool prev_auto_continue;

        double start_time;
        int coord_frame;
        int command_id;
        int command_sub_id;
        int command;
        double param1;
        double param2;
        double param3;
        double param4;
        double param5;
        double param6;
        double param7;
        bool auto_continue;

        int next_coord_frame;
        int next_command_id;
        int next_command;
        double next_param1;
        double next_param2;
        double next_param3;
        double next_param4;
        double next_param5;
        double next_param6;
        double next_param7;
        bool next_auto_continue;

        bool isValid;
        ros::Time receivedTime;
    };

    struct CurrentObjective _receivedObjective;
    struct NavState _currentNavState;

    ros::Rate * _rate;
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
    void cleanup();
  public:
    PathPlanningApp() :
        TaskBase("PathPlanning", "pathplanning") {
    }

    void msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg);
    void msgCurrentObjectiveCallback(const sandshark_msgs::CurrentObjective::ConstPtr & msg);

    void handleSleep();
};

TaskBase *getTaskBase() {
  return new PathPlanningApp();
}

void PathPlanningApp::msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg) {
  _currentNavState.latitude = msg->latitude;
  _currentNavState.longitude = msg->longitude;
  _currentNavState.speed = msg->speed;
  _currentNavState.depth = msg->depth;
  _currentNavState.altitude = msg->altitude;
  _currentNavState.pitch = msg->pitch;
  _currentNavState.roll = msg->roll;
  _currentNavState.bearing = msg->bearing;
  _currentNavState.receivedTime = ros::Time::now();
  _currentNavState.isValid = true;
}

void PathPlanningApp::msgCurrentObjectiveCallback(const sandshark_msgs::CurrentObjective::ConstPtr & msg) {
  _receivedObjective.prev_end_time = msg->prev_end_time;
  _receivedObjective.prev_coord_frame = msg->prev_coord_frame;
  _receivedObjective.prev_command_id = msg->prev_command_id;
  _receivedObjective.prev_command = msg->prev_command;
  _receivedObjective.prev_param1 = msg->prev_param1;
  _receivedObjective.prev_param2 = msg->prev_param2;
  _receivedObjective.prev_param3 = msg->prev_param3;
  _receivedObjective.prev_param4 = msg->prev_param4;
  _receivedObjective.prev_param5 = msg->prev_param5;
  _receivedObjective.prev_param6 = msg->prev_param6;
  _receivedObjective.prev_param7 = msg->prev_param7;
  _receivedObjective.prev_auto_continue = (msg->prev_auto_continue > 0);

  _receivedObjective.start_time = msg->start_time;
  _receivedObjective.coord_frame = msg->coord_frame;
  _receivedObjective.command_id = msg->command_id;
  _receivedObjective.command_sub_id = msg->command_sub_id;
  _receivedObjective.command = msg->command;
  _receivedObjective.param1 = msg->param1;
  _receivedObjective.param2 = msg->param2;
  _receivedObjective.param3 = msg->param3;
  _receivedObjective.param4 = msg->param4;
  _receivedObjective.param5 = msg->param5;
  _receivedObjective.param6 = msg->param6;
  _receivedObjective.param7 = msg->param7;
  _receivedObjective.auto_continue = (msg->auto_continue > 0);

  _receivedObjective.next_coord_frame = msg->next_coord_frame;
  _receivedObjective.next_command_id = msg->next_command_id;
  _receivedObjective.next_command = msg->next_command;
  _receivedObjective.next_param1 = msg->next_param1;
  _receivedObjective.next_param2 = msg->next_param2;
  _receivedObjective.next_param3 = msg->next_param3;
  _receivedObjective.next_param4 = msg->next_param4;
  _receivedObjective.next_param5 = msg->next_param5;
  _receivedObjective.next_param6 = msg->next_param6;
  _receivedObjective.next_param7 = msg->next_param7;
  _receivedObjective.next_auto_continue = (msg->next_auto_continue > 0);

  _receivedObjective.isValid = true;
  _receivedObjective.receivedTime = ros::Time::now();
}

void PathPlanningApp::startupInitCallback() {
  _navSub = _publicNode->subscribe("/navigation/navState", 1, &PathPlanningApp::msgNavigationCallback, this);
  _objSub = _publicNode->subscribe("/objectivecontrol/current_objective", 1,
      &PathPlanningApp::msgCurrentObjectiveCallback, this);

  _cmdPub = _publicNode->advertise<sandshark_msgs::MavlinkObjectiveCommand>("objective_command", 1);

  _receivedObjective.isValid = false;
  _receivedObjective.receivedTime = ros::Time(0);

  _currentNavState.isValid = false;
  _currentNavState.receivedTime = ros::Time(0);

  _rate = new ros::Rate(10);
}

bool PathPlanningApp::doInitialize() {

  return true;
}

void PathPlanningApp::cleanup() {
  //nothing to do...
}

bool PathPlanningApp::doRun() {
  //For this version of ADAPT, pathplanning is just a passthrough,
  //but the prev Objective, next Objective and nav are provided
  //if the command to actionselector needs to be modified
  if (_receivedObjective.isValid) {
    _cmdMsg.start_time = _receivedObjective.start_time;
    _cmdMsg.coord_frame = _receivedObjective.coord_frame;
    _cmdMsg.command_id = _receivedObjective.command_id;
    _cmdMsg.command_sub_id = _receivedObjective.command_sub_id;
    _cmdMsg.command = _receivedObjective.command;
    _cmdMsg.param1 = _receivedObjective.param1;
    _cmdMsg.param2 = _receivedObjective.param2;
    _cmdMsg.param3 = _receivedObjective.param3;
    _cmdMsg.param4 = _receivedObjective.param4;
    _cmdMsg.param5 = _receivedObjective.param5;
    _cmdMsg.param6 = _receivedObjective.param6;
    _cmdMsg.param7 = _receivedObjective.param7;
    _cmdMsg.auto_continue = (_receivedObjective.auto_continue ? 1 : 0);

    _cmdPub.publish(_cmdMsg);
    _receivedObjective.isValid = false;
  }

  return true;
}

void PathPlanningApp::handleSleep() {
  _rate->sleep();
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::PathPlanningApp pp;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) pp, argc, argv);
}

