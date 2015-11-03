#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>

#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/MavlinkObjectiveCommand.h>
#include <sandshark_msgs/DynamicControlCommand.h>
#include <sandshark_msgs/ObjectiveCommand.h>

#include <ros/ros.h>

#define MAVLINK_ALIGNED_FIELDS false
#include <mavlink.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class ActionSelectorApp: public TaskBase {
  private:
    ros::Publisher _dcCmdPub;
    sandshark_msgs::DynamicControlCommand _dcCmdMsg;

    ros::Publisher _objCmdPub;
    sandshark_msgs::ObjectiveCommand _objCmdMsg;

    ros::Subscriber _pathSub;
    ros::Subscriber _navSub;

    struct ObjectiveStartConditions {
        int command_id;
        int command_sub_id;
        double latitude;
        double longitude;

        double subLatitude;
        double subLongitude;
    };

    struct ObjectiveCommand {
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

        bool isValid;
        ros::Time receivedTime;
    };

    struct ObjectiveStartConditions _objStartConditions;
    struct ObjectiveCommand _objCommand;
    struct NavState _currentNavState;

    double _desired_rpm;
    double _maxRPM;
    bool _gpsSurfacing;
    double _gpsSurfacingDistance;
    double _gpsSurfacingTime;
    double _loiterCaptureRadius;
    double getDesiredHeading(double, double, double, double);
    double getDistance(double, double, double, double);

    ros::Rate * _rate;
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
    void cleanup();
  public:
    ActionSelectorApp() :
        TaskBase("ActionSelector", "actionselector") {
    }

    void msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg);
    void msgPathCommandCallback(const sandshark_msgs::MavlinkObjectiveCommand::ConstPtr & msg);

    void handleSleep();
};

TaskBase *getTaskBase() {
  return new ActionSelectorApp();
}

void ActionSelectorApp::msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg) {
  _currentNavState.latitude = msg->latitude;
  _currentNavState.longitude = msg->longitude;
  _currentNavState.speed = msg->speed;
  _currentNavState.depth = msg->depth;
  _currentNavState.altitude = msg->altitude;
  _currentNavState.pitch = msg->pitch;
  _currentNavState.roll = msg->roll;
  _currentNavState.bearing = msg->bearing;
  _currentNavState.epoch = (long long int) msg->epoch;
  _currentNavState.receivedTime = ros::Time::now();
  _currentNavState.isValid = true;
}

void ActionSelectorApp::msgPathCommandCallback(const sandshark_msgs::MavlinkObjectiveCommand::ConstPtr & msg) {
  _objCommand.start_time = msg->start_time;
  _objCommand.command_id = msg->command_id;
  _objCommand.command_sub_id = msg->command_sub_id;
  _objCommand.coord_frame = msg->coord_frame;
  _objCommand.command = msg->command;
  _objCommand.param1 = msg->param1;
  _objCommand.param2 = msg->param2;
  _objCommand.param3 = msg->param3;
  _objCommand.param4 = msg->param4;
  _objCommand.param5 = msg->param5;
  _objCommand.param6 = msg->param6;
  _objCommand.param7 = msg->param7;
  _objCommand.auto_continue = msg->auto_continue;

  _objCommand.isValid = true;
  _objCommand.receivedTime = ros::Time::now();
}

void ActionSelectorApp::startupInitCallback() {
  _dcCmdPub = _publicNode->advertise<sandshark_msgs::DynamicControlCommand>("/dynamiccontrol/command", 10);
  _objCmdPub = _publicNode->advertise<sandshark_msgs::ObjectiveCommand>("/objectivecontrol/command/objective_command",
      10);

  _pathSub = _publicNode->subscribe("/pathplanning/objective_command", 1, &ActionSelectorApp::msgPathCommandCallback,
      this);
  _navSub = _publicNode->subscribe("/navigation/navState", 1, &ActionSelectorApp::msgNavigationCallback, this);

  _privateNode->param("initialRPM", _desired_rpm, double(300.0));
  _privateNode->param("maxRPM", _maxRPM, double(600.0));

  _privateNode->param("gpsSurfacing", _gpsSurfacing, false);
  _privateNode->param("gpsSurfacingDistance", _gpsSurfacingDistance, double(70.0)); //in meters
  _privateNode->param("gpsSurfacingTime", _gpsSurfacingTime, double(90.0)); //in seconds

  _privateNode->param("loiterCaptureRadius", _loiterCaptureRadius, double(7.0)); //in meters

  _rate = new ros::Rate(10);
}

bool ActionSelectorApp::doInitialize() {

  _objStartConditions.command_id = -1;

  _objCommand.isValid = false;

  return true;
}

void ActionSelectorApp::cleanup() {
  //nothing to do...
}

bool ActionSelectorApp::doRun() {
  static bool loiterSprintMode = false;
  if (_currentNavState.isValid && _objCommand.isValid) {
    ros::Time now = ros::Time::now();
    if ((now - _objCommand.receivedTime) < ros::Duration(2.0)) {
      if ((_objCommand.command >= 0) && (_objCommand.command < MAV_CMD_ENUM_END)) {

        if (_objStartConditions.command_id != _objCommand.command_id) {
          _objStartConditions.command_id = _objCommand.command_id;
          _objStartConditions.command_sub_id = _objCommand.command_sub_id;

          _objStartConditions.latitude = _currentNavState.latitude;
          _objStartConditions.longitude = _currentNavState.longitude;

          _objStartConditions.subLatitude = _currentNavState.latitude;
          _objStartConditions.subLongitude = _currentNavState.longitude;
          loiterSprintMode = false;
        } else if (_objStartConditions.command_sub_id != _objCommand.command_sub_id) {
          _objStartConditions.command_sub_id = _objCommand.command_sub_id;

          _objStartConditions.subLatitude = _currentNavState.latitude;
          _objStartConditions.subLongitude = _currentNavState.longitude;
          loiterSprintMode = false;
        }

        switch (_objCommand.command) {
        case (MAV_CMD_NAV_WAYPOINT): {
          double acceptRange = _objCommand.param2;
          double targetRPM = _objCommand.param4;
          double targetLat = _objCommand.param5;
          double targetLng = _objCommand.param6;
          double targetVert = _objCommand.param7;
          double distanceToEnd = getDistance(_currentNavState.latitude, _currentNavState.longitude, targetLat,
              targetLng);
          double desiredHeading = getDesiredHeading(_currentNavState.latitude, _currentNavState.longitude, targetLat,
              targetLng);

          /* * Currently unused. *
           double totalDistanceTraveled = getDistance( _objStartConditions.latitude, _objStartConditions.longitude,
           _currentNavState.latitude, _currentNavState.longitude );

           double subDistanceTraveled = getDistance( _objStartConditions.subLatitude, _objStartConditions.subLongitude,
           _currentNavState.latitude, _currentNavState.longitude );
           */

          ROS_INFO("Distance to end %f, acceptRange %f", distanceToEnd, acceptRange);
          if (distanceToEnd < acceptRange) {
            //Need to pop this objective
            ROS_INFO("Sending pop");
            _objCmdMsg.start_mission = false;
            _objCmdMsg.stop_mission = false;
            _objCmdMsg.pop_objective = true;
            _objCmdMsg.push_objective = false;
            char reason[512];
            snprintf(&reason[0], sizeof(reason), "Popping Objective, distance %f is less than %f", distanceToEnd,
                acceptRange);
            reason[sizeof(reason) - 1] = '\0';
            _objCmdMsg.pop_command_id = _objCommand.command_id;
            _objCmdMsg.pop_commands_sub_id = _objCommand.command_sub_id;
            _objCmdMsg.pop_reason = reason;
            _objCmdPub.publish(_objCmdMsg);
            break;
          }

          _dcCmdMsg.vertical_mode = ((targetVert > 0.0) ? VM_ALTITUDE : VM_DEPTH);
          _dcCmdMsg.vertical_desired = fabs(targetVert);

          _dcCmdMsg.horizontal_mode = HM_HEADING;
          _dcCmdMsg.horizontal_desired = desiredHeading;

          _dcCmdMsg.speed_desired = targetRPM;
          _dcCmdPub.publish(_dcCmdMsg);

          double diff = std::time(0) - (_currentNavState.epoch / 1000);
          if (_gpsSurfacing && diff > _gpsSurfacingTime) {
            //insert gps surface sub-objective
            ROS_INFO("Sending gps surface");
            _objCmdMsg.start_mission = false;
            _objCmdMsg.stop_mission = false;
            _objCmdMsg.pop_objective = false;
            _objCmdMsg.push_objective = true;
            char reason[512];
            snprintf(&reason[0], sizeof(reason), "Pushing Objective, need to surface for gps");
            reason[sizeof(reason) - 1] = '\0';
            _objCmdMsg.push_command_id = _objCommand.command_id;
            _objCmdMsg.push_command = ADAPT_GPS_SURFACE;
            _objCmdMsg.coord_frame = _objCommand.coord_frame;
            _objCmdMsg.auto_continue = true;
            _objCmdMsg.param1 = 0;
            _objCmdMsg.param2 = 0;
            _objCmdMsg.param3 = 0;
            _objCmdMsg.param4 = 0;
            _objCmdMsg.param5 = 0;
            _objCmdMsg.param6 = 0;
            _objCmdMsg.param7 = 0;
            _objCmdMsg.push_reason = reason;
            _objCmdPub.publish(_objCmdMsg);
          }
        }
          break;
        case (MAV_CMD_NAV_LOITER_TIME):
          if (((now.toSec() - _objCommand.start_time) > _objCommand.param1)) {
            //Need to pop this objective
            _objCmdMsg.start_mission = false;
            _objCmdMsg.stop_mission = false;
            _objCmdMsg.pop_objective = true;
            _objCmdMsg.push_objective = false;
            char reason[512];
            snprintf(&reason[0], sizeof(reason), "Popping Objective, duration %f is greater than %f",
                (now.toSec() - _objCommand.start_time), _objCommand.param1);
            reason[sizeof(reason) - 1] = '\0';
            _objCmdMsg.pop_command_id = _objCommand.command_id;
            _objCmdMsg.pop_commands_sub_id = _objCommand.command_sub_id;
            _objCmdMsg.pop_reason = reason;
            _objCmdPub.publish(_objCmdMsg);
            break;
          }
        case (MAV_CMD_NAV_LOITER_UNLIM): {

          double acceptRadius = _objCommand.param3;
          double targetRPM = _objCommand.param4;
          double targetLat = _objCommand.param5;
          double targetLng = _objCommand.param6;
          double distanceToCenter = getDistance(_currentNavState.latitude, _currentNavState.longitude, targetLat,
              targetLng);
          double desiredHeading = getDesiredHeading(_currentNavState.latitude, _currentNavState.longitude, targetLat,
              targetLng);

          //if we are loitering, we should not dive to a depth, since
          //we will float to the surface when we get to the capture radius anyway


          //if we are outside of the capture radius, get back to it on the surface
          if (distanceToCenter > acceptRadius) {
            _dcCmdMsg.vertical_mode = VM_DEPTH;
            _dcCmdMsg.vertical_desired = 0.0;

            _dcCmdMsg.horizontal_mode = HM_HEADING;
            _dcCmdMsg.horizontal_desired = desiredHeading;
            loiterSprintMode = true;
          } else {
            //inside the radius, stay still
            _dcCmdMsg.vertical_mode = VM_ELEVATOR;
            _dcCmdMsg.vertical_desired = 0.0;

            _dcCmdMsg.horizontal_mode = HM_RUDDER;
            _dcCmdMsg.horizontal_desired = 0.0;
            loiterSprintMode = false;
          }
          _dcCmdMsg.speed_desired = loiterSprintMode ? targetRPM : 0.0;
          _dcCmdPub.publish(_dcCmdMsg);
        }
          break;
        case (MAV_CMD_DO_CHANGE_SPEED):
          if ((_objCommand.param3 >= 0) && (_objCommand.param3 <= 100.0)) {
            _desired_rpm = _maxRPM * _objCommand.param3 / 100.0;
          }
          break;
        case (ADAPT_GPS_SURFACE):
          _dcCmdMsg.vertical_mode = VM_DEPTH;
          _dcCmdMsg.vertical_desired = 0;

          _dcCmdMsg.horizontal_mode = HM_RUDDER;
          _dcCmdMsg.horizontal_desired = 10.0;

          _dcCmdMsg.speed_desired = _desired_rpm;

          _dcCmdPub.publish(_dcCmdMsg);

          //If we have a gps hit in the last second, pop objective
          if (std::time(0) - (_currentNavState.epoch / 1000) < 5) {
            //Need to pop this objective
            ROS_INFO("Sending pop");
            _objCmdMsg.start_mission = false;
            _objCmdMsg.stop_mission = false;
            _objCmdMsg.pop_objective = true;
            _objCmdMsg.push_objective = false;
            char reason[512];
            snprintf(&reason[0], sizeof(reason), "Popping Objective, received gps hit");
            reason[sizeof(reason) - 1] = '\0';
            _objCmdMsg.pop_command_id = _objCommand.command_id;
            _objCmdMsg.pop_commands_sub_id = _objCommand.command_sub_id;
            _objCmdMsg.pop_reason = reason;
            _objCmdPub.publish(_objCmdMsg);
          }
          break;
        }
      }
    }
    _currentNavState.isValid = false;
    _objCommand.isValid = false;
  }

  return true;
}

void ActionSelectorApp::handleSleep() {
  _rate->sleep();
}

double ActionSelectorApp::getDistance(double startLat, double startLng, double targetLat, double targetLng) {
  static const double earthRadius = 6371000; //m

  double deltaLat = (targetLat - startLat) / 180.0 * M_PI;
  double deltaLng = (targetLng - startLng) / 180.0 * M_PI;

  double sindLatSquared = sin(deltaLat / 2.0) * sin(deltaLat / 2.0);
  double sindLngSquared = sin(deltaLng / 2.0) * sin(deltaLng / 2.0);

  double a = sindLatSquared + cos(startLat / 180.0 * M_PI) * cos(targetLat / 180.0 * M_PI) * sindLngSquared;
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return (earthRadius * c);

}

double ActionSelectorApp::getDesiredHeading(double startLat, double startLng, double targetLat, double targetLng) {
  double deltaLong = (targetLng - startLng) / 180.0 * M_PI;
  return (atan2(sin(deltaLong) * cos(targetLat / 180.0 * M_PI),
      cos(startLat / 180.0 * M_PI) * sin(targetLat / 180.0 * M_PI)
          - sin(startLat / 180.0 * M_PI) * cos(targetLat / 180.0 * M_PI) * cos(deltaLong))) / M_PI * 180.0;
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::ActionSelectorApp as;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) as, argc, argv);
}

