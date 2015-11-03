#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>

#include <math.h>

#include <ros/ros.h>
#include <sandshark_msgs/CurrentObjective.h>
#include <sandshark_msgs/ObjectiveControlStatus.h>
#include <sandshark_msgs/MavlinkWaypointList.h>
#include <sandshark_msgs/MavlinkPolygon.h>

#include <sandshark_msgs/ObjectiveCommand.h>
#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/ApplicationAbort.h>
#include <sandshark_msgs/LEDCommand.h>
#include <sandshark_msgs/LEDStatus.h>

#include <sandshark_msgs/ApplicationRestartCommand.h>
#include <sandshark_msgs/ElapsedMissionTime.h>

#define MAVLINK_ALIGNED_FIELDS false
#include <mavlink.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class ObjectiveControlApp: public TaskBase {
  private:
    ros::Publisher _curObjPub;
    sandshark_msgs::CurrentObjective _curObjMsg;

    ros::Publisher _objStatPub;
    sandshark_msgs::ObjectiveControlStatus _objStatMsg;

    ros::Publisher _mavWaypointsPub;
    sandshark_msgs::MavlinkWaypointList _mavWaypointsMsg;

    ros::Publisher _redLEDPub;
    ros::Publisher _amberLEDPub;
    ros::Publisher _greenLEDPub;
    ros::Publisher _strobeLEDPub;

    ros::Publisher _elapsedTimePub;
    sandshark_msgs::ElapsedMissionTime _elapsedTimeMsg;

    ros::Publisher _tcRestartPub;
    sandshark_msgs::ApplicationRestartCommand _tcRestartMsg;

    ros::Subscriber _ledSub;
    ros::Subscriber _navSub;
    ros::Subscriber _mavWaypointSub;
    ros::Subscriber _mavPolygonSub;
    ros::Subscriber _objCommandSub;
    ros::Subscriber _abortSub;

    uint8_t _redStatus;
    uint8_t _amberStatus;
    uint8_t _greenStatus;

    bool _receivedWaypointList;
    struct Waypoint {
        int16_t command_id;
        uint8_t coord_frame;
        uint16_t command;
        double param1;
        double param2;
        double param3;
        double param4;
        double param5;
        double param6;
        double param7;
        bool auto_continue;
    };

    class Objective {
      private:
        struct Waypoint _way;
        vector<Objective *> _subObjectives;

        int _id;
        int _subId;
        bool _paused;
        bool _ran;
        bool _started;

        ros::Time _start_time;
        ros::Time _endTime;

        Objective * _parent;

        unsigned int _nextChildID;
      public:
        explicit Objective(int id, Objective * parent = NULL) :
            _id(id), _started(false), _parent(parent) {
          _way.command = INVALID_OBJECTIVE;
          if (_parent) {
            _subId = _parent->getNextChildID();
          } else {
            _subId = INVALID_OBJECTIVE;
          }
          _nextChildID = 0;
        }

        int getCommandID() const {
          return _id;
        }
        int getCommandSubID() const {
          if (!_subObjectives.empty()) {
            return _subObjectives[0]->getCommandSubID();
          }
          return _subId;
        }

        unsigned int getNextChildID() {
          return ++_nextChildID;
        }

        double getParam7() const {
          return _way.param7;
        }

        void setWaypoint(struct Waypoint & wp) {
          memcpy(&(_way), &(wp), sizeof(_way));
        }

        bool hasNextObjective() const {
          return !_subObjectives.empty();
        }

        bool started() {
          return _started;
        }

        void fillPreviousObjective(sandshark_msgs::CurrentObjective & acoMsg);
        void fillCurrentObjective(sandshark_msgs::CurrentObjective & acoMsg);
        void fillNextObjective(sandshark_msgs::CurrentObjective & acoMsg);

        void fillMavlinkMessage(sandshark_msgs::MavlinkWaypointList & mavList);

        void addSubObjective(struct Waypoint & wp);
        bool pop_objective();
        void markComplete();
        void markStart();
    };

    bool _firstRun;
    bool _start_mission;
    bool _stop_mission;
    bool _abort;
    bool _missionAborted;
    std::string _abortReason;

    double _elapsedMissionSeconds;
    ros::Time _lastRunTime;

    struct PopInfo {
        int command_id;
        int subID;
        std::string reason;
    };
    deque<struct PopInfo> _pop_objectives;

    deque<struct Waypoint> _waypointsToPush;

    int _current_point;
    int _recvCurrentWaypoint;
    std::vector<Objective *> _currentList;
    std::vector<struct Waypoint> _receivedList;

    bool _mission_running;
    bool _have_nav;
    ros::Time _lastMeasuredTime;

    bool _mission_started;
    double _depth;

    void insertDiveWaypoints(Objective * obj) {
    }
    void insertSurfacingWaypoints(Objective * obj) {
    }
    void clearCurrentList();
    void addWaypointToList(struct Waypoint & wpoint);
    void sendReadyLED();
    void sendRunningLED();
    void sendAbortLED();

    ros::Rate * _rate;
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
    void cleanup();
  public:
    ObjectiveControlApp() :
        TaskBase("ObjectiveControl", "objectivecontrol") {
    }

    void msgLEDCallback(const sandshark_msgs::LEDStatus::ConstPtr & msg);
    void msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg);
    void msgMavLinkWaypointCallback(const sandshark_msgs::MavlinkWaypointList::ConstPtr & msg);
    void msgMavLinkPolygonCallback(const sandshark_msgs::MavlinkPolygon::ConstPtr & msg);
    void msgObjectiveCommandCallback(const sandshark_msgs::ObjectiveCommand::ConstPtr & msg);
    void msgAbortCallback(const sandshark_msgs::ApplicationAbort::ConstPtr & msg);

    void handleSleep();
};

TaskBase *getTaskBase() {
  return new ObjectiveControlApp();
}

void ObjectiveControlApp::msgLEDCallback(const sandshark_msgs::LEDStatus::ConstPtr & msg) {
  _redStatus = msg->red_status;
  _amberStatus = msg->amber_status;
  _greenStatus = msg->green_status;
}

void ObjectiveControlApp::msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg) {
  _have_nav = true;
  _depth = msg->depth;
  _lastMeasuredTime = ros::Time::now();
}

void ObjectiveControlApp::msgMavLinkWaypointCallback(const sandshark_msgs::MavlinkWaypointList::ConstPtr & msg) {
  if (msg->number_of_points != msg->command.size()) {
    return;
  }

  _receivedList.clear();
  struct Waypoint waypoint;
  for (unsigned int index = 0; index < msg->number_of_points; ++index) {
    waypoint.coord_frame = msg->coord_frame[index];
    waypoint.command = msg->command[index];
    waypoint.param1 = msg->param1[index];
    waypoint.param2 = msg->param2[index];
    waypoint.param3 = msg->param3[index];
    waypoint.param4 = msg->param4[index];
    waypoint.param5 = msg->param5[index];
    waypoint.param6 = msg->param6[index];
    waypoint.param7 = msg->param7[index];
    waypoint.auto_continue = msg->auto_continue[index];
    _receivedList.push_back(waypoint);
  }

  _recvCurrentWaypoint = msg->current_point;
  _receivedWaypointList = true;

  //try to get the vehicle into a running state
  _missionAborted = false;
}

void ObjectiveControlApp::msgMavLinkPolygonCallback(const sandshark_msgs::MavlinkPolygon::ConstPtr & msg) {

  if (msg->polygon_mode != sandshark_msgs::MavlinkPolygon::BOUNDING_POLYGON) {
    ROS_WARN("Only bounding polygons supported! Type %d, not implemented", msg->polygon_mode);
    return;
  }

  if (msg->number_of_points != msg->poly_lat.size() || msg->number_of_points != msg->poly_lon.size()) {
    ROS_WARN("Polygon's number of points do not match the number of points"
        " given. Points: %d. Lat Points: %d. Lon Points: %d.", msg->number_of_points, (int) msg->poly_lat.size(),
        (int) msg->poly_lon.size());
    return;
  }

  double xSum = 0, ySum = 0, zSum = 0, x = 0, y = 0, z = 0;

  double centerLat, centerLon;

  //convert to cartesian coordinates
  for (int i = 0; i < msg->number_of_points; i++) {

    //convert to radians
    double lat = (msg->poly_lat[i] / 10000000) * M_PI / 180.0;
    double lon = (msg->poly_lon[i] / 10000000) * M_PI / 180.0;

    xSum += cos(lat) * cos(lon);
    ySum += cos(lat) * sin(lon);
    zSum += sin(lat);
  }

  x = xSum / msg->number_of_points;
  y = ySum / msg->number_of_points;
  z = zSum / msg->number_of_points;

  if (abs(x) < pow(10, -9) && abs(y) < pow(10, -9) && abs(z) < pow(10, -9)) {
    // geographic midpoint is the center of the earth, so any midpoint
    // is correct. Set it to (0,0)
    centerLat = 0.0;
    centerLon = 0.0;
  } else {
    centerLon = atan2(y, x);
    double hyp = sqrt(x * x + y * y);
    centerLat = atan2(z, hyp);

    //convert to degrees
    centerLat = centerLat * 180.0 / M_PI;
    centerLon = centerLon * 180.0 / M_PI;
  }

  ROS_INFO("The polygon coordinates are:");

  for (int i = 0; i < msg->number_of_points; i++) {
    ROS_INFO("(%f, %f)", msg->poly_lat[i], msg->poly_lon[i]);
  }

  ROS_INFO("The center point is: (%f,%f).", centerLat, centerLon);

  //now make a loiter waypoint with the lat/lon center point

  _receivedList.clear();
  struct Waypoint waypoint;
  waypoint.coord_frame = MAV_FRAME_GLOBAL; // global frame:
                                           // x=lat, y=lon, z=altitude/depth
  waypoint.command = MAV_CMD_NAV_WAYPOINT; //no time limit
  waypoint.param1 = 0.0; // empty
  waypoint.param2 = 10.0; // accept radius in meters
  waypoint.param3 = 10.0; // empty
  waypoint.param4 = 0.0; // desired yaw - NOT USED
  waypoint.param5 = centerLat; // latitude
  waypoint.param6 = centerLon; // longitude
  waypoint.param7 = 0.0; // 0 depth mission
  waypoint.auto_continue = 1; // auto continue true
  _receivedList.push_back(waypoint);

  _recvCurrentWaypoint = 0;
  _receivedWaypointList = true;

  //try to get the vehicle into a running state
  _missionAborted = false;
}

void ObjectiveControlApp::msgObjectiveCommandCallback(const sandshark_msgs::ObjectiveCommand::ConstPtr & msg) {
  _mission_started = true;
  _start_mission = msg->start_mission;
  _stop_mission = msg->stop_mission;

  if (msg->pop_objective) {
    ROS_INFO("Received message to pop objective. ID %d sub %d Reason was %s", msg->pop_command_id,
        msg->pop_commands_sub_id, msg->pop_reason.c_str());
    struct PopInfo pop;
    pop.command_id = msg->pop_command_id;
    pop.subID = msg->pop_commands_sub_id;
    pop.reason = msg->pop_reason;
    _pop_objectives.push_front(pop);
  }

  if (msg->push_objective) {
    struct Waypoint waypoint;
    waypoint.command_id = msg->push_command_id;
    waypoint.coord_frame = msg->coord_frame;
    waypoint.command = msg->push_command;
    waypoint.param1 = msg->param1;
    waypoint.param2 = msg->param2;
    waypoint.param3 = msg->param3;
    waypoint.param4 = msg->param4;
    waypoint.param5 = msg->param5;
    waypoint.param6 = msg->param6;
    waypoint.param7 = msg->param7;
    waypoint.auto_continue = msg->auto_continue;

    _waypointsToPush.push_back(waypoint);
  }

  //try to get the vehicle into a running state
  _missionAborted = false;
}

void ObjectiveControlApp::sendReadyLED() {
  sandshark_msgs::LEDCommand msg;

  if (_greenStatus != sandshark_msgs::LEDCommand::COMMAND_ON) {
    msg.command = sandshark_msgs::LEDCommand::COMMAND_ON;
    _greenLEDPub.publish(msg);
  }

  //msg.command = sandshark_msgs::LEDCommand::COMMAND_OFF;
  //_strobeLEDPub.publish(msg);

  if (_redStatus != sandshark_msgs::LEDCommand::COMMAND_OFF) {
    msg.command = sandshark_msgs::LEDCommand::COMMAND_OFF;
    _redLEDPub.publish(msg);
  }
}

void ObjectiveControlApp::sendRunningLED() {
  sandshark_msgs::LEDCommand msg;

  if (_greenStatus != sandshark_msgs::LEDCommand::COMMAND_BLINK_FAST) {
    msg.command = sandshark_msgs::LEDCommand::COMMAND_BLINK_FAST;
    _greenLEDPub.publish(msg);
  }

  //msg.command = sandshark_msgs::LEDCommand::COMMAND_BLINK_FAST;
  //_strobeLEDPub.publish(msg);

  if (_redStatus != sandshark_msgs::LEDCommand::COMMAND_OFF) {
    msg.command = sandshark_msgs::LEDCommand::COMMAND_OFF;
    _redLEDPub.publish(msg);
  }
}

void ObjectiveControlApp::sendAbortLED() {
  sandshark_msgs::LEDCommand msg;
  if (_redStatus != sandshark_msgs::LEDCommand::COMMAND_ON) {
    msg.command = sandshark_msgs::LEDCommand::COMMAND_ON;
    _redLEDPub.publish(msg);
  }

  //msg.command = sandshark_msgs::LEDCommand::COMMAND_OFF;
  //_strobeLEDPub.publish(msg);

  if (_greenStatus != sandshark_msgs::LEDCommand::COMMAND_OFF) {
    msg.command = sandshark_msgs::LEDCommand::COMMAND_OFF;
    _greenLEDPub.publish(msg);
  }
}

void ObjectiveControlApp::msgAbortCallback(const sandshark_msgs::ApplicationAbort::ConstPtr & msg) {
//   ABORT!!!
  _abort = msg->abort;
  _abortReason = msg->reason;

}

void ObjectiveControlApp::startupInitCallback() {
  _curObjPub = _publicNode->advertise<sandshark_msgs::CurrentObjective>("current_objective", 1);
  _objStatPub = _publicNode->advertise<sandshark_msgs::ObjectiveControlStatus>("objective_status", 1);
  _mavWaypointsPub = _publicNode->advertise<sandshark_msgs::MavlinkWaypointList>("waypoint_list", 1);

  _elapsedTimePub = _publicNode->advertise<sandshark_msgs::ElapsedMissionTime>("elapsed_time", 1);

  _tcRestartPub = _publicNode->advertise<sandshark_msgs::ApplicationRestartCommand>("/health/control/TailconeDriver", 1);

  _navSub = _publicNode->subscribe("/navigation/navState", 1, &ObjectiveControlApp::msgNavigationCallback, this);
  _mavWaypointSub = _publicNode->subscribe("command/received_waypoint_list", 1,
      &ObjectiveControlApp::msgMavLinkWaypointCallback, this);
  _mavPolygonSub = _publicNode->subscribe("command/received_polygon", 1,
      &ObjectiveControlApp::msgMavLinkPolygonCallback, this);
  _objCommandSub = _publicNode->subscribe("command/objective_command", 1,
      &ObjectiveControlApp::msgObjectiveCommandCallback, this);
  _abortSub = _publicNode->subscribe("abort", 1, &ObjectiveControlApp::msgAbortCallback, this);

  _redLEDPub = _publicNode->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/red", 1);
  _greenLEDPub = _publicNode->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/green", 1);
  _amberLEDPub = _publicNode->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/amber", 1);
  _strobeLEDPub = _publicNode->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/strobe", 1);

  _ledSub = _publicNode->subscribe("/ledcontrol/ledstatus", 1, &ObjectiveControlApp::msgLEDCallback, this);
  _rate = new ros::Rate(1);
}

bool ObjectiveControlApp::doInitialize() {
  _receivedWaypointList = false;
  _start_mission = false;
  _stop_mission = false;
  _firstRun = true;
  while (!_pop_objectives.empty()) {
    _pop_objectives.pop_back();
  }
  //_havePushWaypoints = false;
  _have_nav = false;
  _receivedWaypointList = false;
  _mission_running = false;
  _abort = false;
  _missionAborted = false;
  _abortReason = "";

  _currentList.clear();
  _receivedList.clear();

  while (!_waypointsToPush.empty()) {
    _waypointsToPush.pop_front();
  }

  _lastRunTime = ros::Time::now();
  _elapsedMissionSeconds = 0.0;

  _mission_started = false;

  return true;
}

void ObjectiveControlApp::addWaypointToList(struct Waypoint & wpoint) {
  Objective *prevWay = NULL;
  if (!_currentList.empty()) {
    prevWay = _currentList[(int) _currentList.size() - 1];
  }

  bool onSurface = true;
  if (prevWay && (fabs(prevWay->getParam7()) > 0.5)) {
    onSurface = false;
  }

  if (onSurface && (fabs(wpoint.param7) > 0.5)) {
    insertDiveWaypoints(prevWay);
  }

  if (!onSurface && (fabs(wpoint.param7) < 0.5)) {
    insertSurfacingWaypoints(prevWay);
  }

  Objective *obj = new Objective((int) _currentList.size());
  obj->setWaypoint(wpoint);
  _currentList.push_back(obj);
}

void ObjectiveControlApp::clearCurrentList() {
  while (!_waypointsToPush.empty()) {
    _waypointsToPush.pop_front();
  }

  _currentList.clear();
}

void ObjectiveControlApp::cleanup() {
  //nothing to do...
}

bool ObjectiveControlApp::doRun() {

  ros::Time now = ros::Time::now();

  if (_mission_running) {
    ros::Duration elapsed = now - _lastRunTime;
    _elapsedMissionSeconds += elapsed.toSec();
  } else {
    _elapsedMissionSeconds = 0.0;
  }

  _elapsedTimeMsg.seconds_elapsed = _elapsedMissionSeconds;
  _elapsedTimePub.publish(_elapsedTimeMsg);

  _lastRunTime = now;

  if (_receivedWaypointList) {
    clearCurrentList();
    _current_point = 0;
    for (std::vector<struct Waypoint>::iterator iter = _receivedList.begin(); iter != _receivedList.end(); ++iter) {
      addWaypointToList((*iter));
    }
    _receivedWaypointList = false;
  }

  //Only check pop objectives if we are not past the end of the current objective list
  //This keeps us safe if the same pop command gets sent twice
  if (_current_point < (int) _currentList.size()) {
    while ((!_pop_objectives.empty()) && (!_currentList.empty())) {
      struct PopInfo popInfo = _pop_objectives.back();
      ROS_INFO("Popping Objective. ID %d sub %d Reason was %s", popInfo.command_id, popInfo.subID,
          popInfo.reason.c_str());
      if (popInfo.command_id == INVALID_OBJECTIVE
          || ((popInfo.command_id == _currentList[_current_point]->getCommandID())
              && (popInfo.subID == _currentList[_current_point]->getCommandSubID()))) {
        if (!_currentList[_current_point]->pop_objective()) {
          //No subObjectives left, move on the next point
          _currentList[_current_point]->markComplete();
          _current_point++;
          if (_current_point < (int) _currentList.size()) {
            _currentList[_current_point]->markStart();
          }
        }
      } else {
        ROS_ERROR("Failed to pop Objective. ID %d sub %d Reason was %s Current ID %d SubID %d", popInfo.command_id,
            popInfo.subID, popInfo.reason.c_str(), _currentList[_current_point]->getCommandID(),
            _currentList[_current_point]->getCommandSubID());
      }
      _pop_objectives.pop_back();
    }
  }

  while (!_pop_objectives.empty()) {
    //Throw away any extras
    _pop_objectives.pop_back();
  }

  bool validCurrent = _current_point < (int) _currentList.size();

  while (!_waypointsToPush.empty()) {
    struct Waypoint pushPoint = _waypointsToPush.front();
    if (validCurrent
        && ((pushPoint.command_id != INVALID_OBJECTIVE)
            || (pushPoint.command_id == _currentList[_current_point]->getCommandID()))) {
      _currentList[_current_point]->addSubObjective(pushPoint);
    } else {
      ROS_ERROR("Failed to push way, desired command_id = %d current is %s, ID = %d", pushPoint.command_id,
          (validCurrent ? "valid" : "invalid"), (validCurrent ? _currentList[_current_point]->getCommandID() : -2));
    }
    _waypointsToPush.pop_front();
  }

  if (_abort) {
    _mission_running = false;
    _missionAborted = true;
    ROS_WARN("Mission Aborted!! Reason was %s", _abortReason.c_str());
    //What else should we do here? Maybe a wrapup?
  }
  _abort = false;

  if (!validCurrent && _mission_running) {
    _mission_running = false;
    ROS_INFO("Mission Complete!");
  }

  if (_have_nav && _mission_running && _current_point < (int) _currentList.size()) {
    if ((_current_point - 1) > 0) {
      _currentList[_current_point - 1]->fillPreviousObjective(_curObjMsg);
    } else {
      _curObjMsg.prev_command = INVALID_OBJECTIVE;
    }

    _currentList[_current_point]->fillCurrentObjective(_curObjMsg);
    if (_currentList[_current_point]->hasNextObjective()) {
      _currentList[_current_point]->fillNextObjective(_curObjMsg);
    } else if ((_current_point + 1) < (int) _currentList.size()) {
      _currentList[_current_point + 1]->fillNextObjective(_curObjMsg);
    } else {
      _curObjMsg.next_command = INVALID_OBJECTIVE;
    }

    _curObjPub.publish(_curObjMsg);
  }

  _objStatMsg.mission_running = _mission_running;
  _objStatMsg.have_nav = false;
  if (_have_nav && ((now - _lastMeasuredTime) < ros::Duration(1.0))) {
    _objStatMsg.have_nav = true;
  }
  _objStatMsg.current_waypoint = _current_point;
  _objStatMsg.is_aborted = _missionAborted;
  _objStatPub.publish(_objStatMsg);

  _mavWaypointsMsg.number_of_points = _currentList.size();
  _mavWaypointsMsg.current_point = _current_point;

  _mavWaypointsMsg.coord_frame.clear();
  _mavWaypointsMsg.command.clear();
  _mavWaypointsMsg.param1.clear();
  _mavWaypointsMsg.param2.clear();
  _mavWaypointsMsg.param3.clear();
  ;
  _mavWaypointsMsg.param4.clear();
  _mavWaypointsMsg.param5.clear();
  _mavWaypointsMsg.param6.clear();
  _mavWaypointsMsg.param7.clear();
  _mavWaypointsMsg.auto_continue.clear();

  for (vector<Objective *>::iterator iter = _currentList.begin(); iter != _currentList.end(); ++iter) {
    (*iter)->fillMavlinkMessage(_mavWaypointsMsg);
  }
  _mavWaypointsPub.publish(_mavWaypointsMsg);

  //check to see if there are current waypoints and we should start a mission
  if (_start_mission && _currentList.size() - _current_point > 0) {
    _missionAborted = false;
    _mission_running = true;
    _start_mission = false;

    _tcRestartMsg.restart = true;
    _tcRestartPub.publish(_tcRestartMsg);

    //set markStart on the curret objective
    _currentList[_current_point]->markStart();

    //as soon as we start a mission
  }

  //Actually stop the mission
  if (_stop_mission) {
    _mission_running = false;
    _stop_mission = false;
  }

  if (_mission_running) {
    sendRunningLED();
  } else if (!_mission_running && !_missionAborted) {
    sendReadyLED();
  } else if (_missionAborted) {
    sendAbortLED();
  }

  //strobe the led if we have started at least 1 mission and we are no
  //deeper than 1 meter
  sandshark_msgs::LEDCommand strobeMsg;
  if (_mission_started && _depth < 1.0) {
    strobeMsg.command = sandshark_msgs::LEDCommand::COMMAND_BLINK_SLOW;
  } else {
    strobeMsg.command = sandshark_msgs::LEDCommand::COMMAND_OFF;
  }
  _strobeLEDPub.publish(strobeMsg);

  return true;
}

void ObjectiveControlApp::handleSleep() {
  _rate->sleep();
}

void ObjectiveControlApp::Objective::addSubObjective(struct Waypoint & wp) {
  Objective *sub = new Objective(_id, this);
  sub->setWaypoint(wp);
  _subObjectives.push_back(sub);
}

bool ObjectiveControlApp::Objective::pop_objective() {
  if (!_subObjectives.empty()) {
    delete _subObjectives.front();
    _subObjectives.erase(_subObjectives.begin());
    return true;
  }
  return false;
}

void ObjectiveControlApp::Objective::markComplete() {
  _endTime = ros::Time::now();
}

void ObjectiveControlApp::Objective::markStart() {
  _start_time = ros::Time::now();
}

void ObjectiveControlApp::Objective::fillPreviousObjective(sandshark_msgs::CurrentObjective & acoMsg) {
  acoMsg.prev_end_time = _endTime.toSec();
  acoMsg.prev_coord_frame = _way.coord_frame;
  acoMsg.prev_command_id = _id;
  acoMsg.prev_command = _way.command;
  acoMsg.prev_param1 = _way.param1;
  acoMsg.prev_param2 = _way.param2;
  acoMsg.prev_param3 = _way.param3;
  acoMsg.prev_param4 = _way.param4;
  acoMsg.prev_param5 = _way.param5;
  acoMsg.prev_param6 = _way.param6;
  acoMsg.prev_param7 = _way.param7;
  acoMsg.prev_auto_continue = _way.auto_continue;
}

void ObjectiveControlApp::Objective::fillNextObjective(sandshark_msgs::CurrentObjective & acoMsg) {
  if ((int) _subObjectives.size() > 1) {
    _subObjectives[1]->fillNextObjective(acoMsg);
  } else if (_parent != NULL) {
    acoMsg.next_coord_frame = _way.coord_frame;
    acoMsg.next_command = _way.command;
    acoMsg.next_command_id = _id;
    acoMsg.next_param1 = _way.param1;
    acoMsg.next_param2 = _way.param2;
    acoMsg.next_param3 = _way.param3;
    acoMsg.next_param4 = _way.param4;
    acoMsg.next_param5 = _way.param5;
    acoMsg.next_param6 = _way.param6;
    acoMsg.next_param7 = _way.param7;
    acoMsg.next_auto_continue = _way.auto_continue;
  } else {
    acoMsg.next_command = INVALID_OBJECTIVE;
  }
}

void ObjectiveControlApp::Objective::fillCurrentObjective(sandshark_msgs::CurrentObjective & acoMsg) {
  if (!_subObjectives.empty()) {
    if (!_subObjectives[0]->started()) {
      _subObjectives[0]->markStart();
    }
    _subObjectives[0]->fillCurrentObjective(acoMsg);
  } else {
    acoMsg.start_time = _start_time.toSec();
    acoMsg.coord_frame = _way.coord_frame;
    acoMsg.command_id = _id;
    acoMsg.command_sub_id = _subId;
    acoMsg.command = _way.command;
    acoMsg.param1 = _way.param1;
    acoMsg.param2 = _way.param2;
    acoMsg.param3 = _way.param3;
    acoMsg.param4 = _way.param4;
    acoMsg.param5 = _way.param5;
    acoMsg.param6 = _way.param6;
    acoMsg.param7 = _way.param7;
    acoMsg.auto_continue = _way.auto_continue;
  }
}

void ObjectiveControlApp::Objective::fillMavlinkMessage(sandshark_msgs::MavlinkWaypointList & mavListMsg) {
  mavListMsg.coord_frame.push_back(_way.coord_frame);
  mavListMsg.command.push_back(_way.command);
  mavListMsg.param1.push_back(_way.param1);
  mavListMsg.param2.push_back(_way.param2);
  mavListMsg.param3.push_back(_way.param3);
  mavListMsg.param4.push_back(_way.param4);
  mavListMsg.param5.push_back(_way.param5);
  mavListMsg.param6.push_back(_way.param6);
  mavListMsg.param7.push_back(_way.param7);
  mavListMsg.auto_continue.push_back(_way.auto_continue);
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::ObjectiveControlApp oc;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) oc, argc, argv);
}

