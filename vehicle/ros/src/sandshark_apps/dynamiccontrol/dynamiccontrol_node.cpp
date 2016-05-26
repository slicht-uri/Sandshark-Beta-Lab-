#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/PIDController.h>

#include <ros/ros.h>

#include <sandshark_msgs/DynamicControlCommand.h>
#include <sandshark_msgs/DynamicControlGains.h>
#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/TailconeState.h>
#include <sandshark_msgs/Speed.h>
#include <sandshark_msgs/UpdateDCGain.h>

using namespace std;
namespace bluefin {
namespace sandshark {

//Used in sorting the rpm speed map
bool SpeedVCompare(std::pair<int,double> i, std::pair<int,double> j) {
  return (i.first < j.first);
}

class DynamicControlApp: public TaskBase {
  private:
    ros::Subscriber _commandSub;
    ros::Subscriber _navSub;
    ros::Subscriber _curTailSub;

    ros::ServiceServer _gainService;

    ros::Publisher _tailCmdPub;
    sandshark_msgs::TailconeState _tailCmdMsg;

    ros::Publisher _speedPub;
    sandshark_msgs::Speed _speedMsg;

    ros::Publisher _gainPub;
    sandshark_msgs::DynamicControlGains _gainMsg;

    PIDController *_depthPID;
    PIDController *_pitchPID;
    PIDController *_headingPID;

    enum DynamicControlVerticalMode _vertical_mode;
    double _verticalDesired;
    enum DynamicControlHorizontalMode _horizontal_mode;
    double _horizontalDesired;
    double _rpmDesired;

    double _depthMeasured;
    double _altitudeMeasured;
    double _pitchMeasured;
    double _bearingMeasured;
    double _rollMeasured;

    ros::Time _lastCommandedTime;
    ros::Time _lastMeasuredTime;
    ros::Time _lastRunTime;

    double _rpmToMetersPerSec;
    bool _correctTailconePerRoll;

    // map of double to double is not supported for some reason
    std::map<std::string, std::string> _speeds;
    std::vector<std::pair<double, double> > _speedV;

    ros::Rate * _rate;
    double getMPSfromRPM(double rpm);
    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    DynamicControlApp() :
        TaskBase("DynamicControl", "dynamiccontrol") {
    }

    void msgCommandCallback(const sandshark_msgs::DynamicControlCommand::ConstPtr & msg);
    void msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg);
    void msgCurrentTailconeCallback(const sandshark_msgs::TailconeState::ConstPtr & msg);
    bool handleGainUpdate(sandshark_msgs::UpdateDCGain::Request &req, sandshark_msgs::UpdateDCGain::Response &res);

    void handleSleep();

    //bool SpeedVCompare(std::pair<int,double> i, std::pair<int,double> j);
};

TaskBase *getTaskBase() {
  return new DynamicControlApp();
}

void DynamicControlApp::msgCommandCallback(const sandshark_msgs::DynamicControlCommand::ConstPtr & msg) {
  if ((msg->vertical_mode > VM_INVALID_MIN) && (msg->vertical_mode < VM_INVALID_MAX)) {
    _vertical_mode = static_cast<enum DynamicControlVerticalMode>(msg->vertical_mode);
  } else {
    _vertical_mode = VM_INVALID_MIN;
  }
  _verticalDesired = msg->vertical_desired;

  if ((msg->horizontal_mode > HM_INVALID_MIN) && (msg->horizontal_mode < HM_INVALID_MAX)) {
    _horizontal_mode = static_cast<enum DynamicControlHorizontalMode>(msg->horizontal_mode);
  } else {
    _horizontal_mode = HM_INVALID_MIN;
  }
  _horizontalDesired = msg->horizontal_desired;

  _rpmDesired = msg->speed_desired;  //we have no speed mode

  _lastCommandedTime = ros::Time::now();
}

void DynamicControlApp::msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg) {
  _depthMeasured = msg->depth;
  _altitudeMeasured = msg->altitude;
  _pitchMeasured = msg->pitch;
  _bearingMeasured = msg->bearing;
  _rollMeasured = msg->roll;

  _lastMeasuredTime = ros::Time::now();
}

void DynamicControlApp::msgCurrentTailconeCallback(const sandshark_msgs::TailconeState::ConstPtr & msg) {
// speed should be speed over ground, so we have to take pitch into account
  _speedMsg.speed_estimate = getMPSfromRPM(msg->thruster) * cos(_pitchMeasured * (M_PI / 180.0));
  _speedPub.publish(_speedMsg);
}

bool DynamicControlApp::handleGainUpdate(sandshark_msgs::UpdateDCGain::Request &req,
    sandshark_msgs::UpdateDCGain::Response &res) {
  if ((req.updateGainID > GU_INVALID_MIN) && (req.updateGainID < GU_INVALID_MAX)) {
    enum DynamicControlGainUpdate guID = static_cast<enum DynamicControlGainUpdate>(req.updateGainID);
    switch (guID) {
    case (GU_HEADING_P):
      _privateNode->setParam("headingpgain", req.value);
      _headingPID->setGains(req.value, _headingPID->getIntGain(), _headingPID->getDerGain());
      _headingPID->reset();
      break;
    case (GU_HEADING_I):
      _privateNode->setParam("headingigain", req.value);
      _headingPID->setGains(_headingPID->getPropGain(), req.value, _headingPID->getDerGain());
      _headingPID->reset();
      break;
    case (GU_HEADING_D):
      _privateNode->setParam("headingdgain", req.value);
      _headingPID->setGains(_headingPID->getPropGain(), _headingPID->getIntGain(), req.value);
      _headingPID->reset();
      break;
    case (GU_DEPTH_P):
      _privateNode->setParam("depthpgain", req.value);
      _depthPID->setGains(req.value, _depthPID->getIntGain(), _depthPID->getDerGain());
      _depthPID->reset();
      break;
    case (GU_DEPTH_I):
      _privateNode->setParam("depthigain", req.value);
      _depthPID->setGains(_depthPID->getPropGain(), req.value, _depthPID->getDerGain());
      _depthPID->reset();
      break;
    case (GU_DEPTH_D):
      _privateNode->setParam("depthdgain", req.value);
      _depthPID->setGains(_depthPID->getPropGain(), _depthPID->getIntGain(), req.value);
      _depthPID->reset();
      break;
    case (GU_PITCH_P):
      _privateNode->setParam("pitchpgain", req.value);
      _pitchPID->setGains(req.value, _pitchPID->getIntGain(), _pitchPID->getDerGain());
      _pitchPID->reset();
      break;
    case (GU_PITCH_I):
      _privateNode->setParam("pitchigain", req.value);
      _pitchPID->setGains(_pitchPID->getPropGain(), req.value, _pitchPID->getDerGain());
      _pitchPID->reset();
      break;
    case (GU_PITCH_D):
      _privateNode->setParam("pitchdgain", req.value);
      _pitchPID->setGains(_pitchPID->getPropGain(), _pitchPID->getIntGain(), req.value);
      _pitchPID->reset();
      break;
    case (GU_RPMTOSPEED):
      _rpmToMetersPerSec = req.value;
      _privateNode->setParam("rpm_to_speed", _rpmToMetersPerSec);
      break;

    case (GU_FLUOR):
    case (GU_INVALID_MIN):
    case (GU_INVALID_MAX):
      //we shouldn't be here
      break;
    }
    system("rosparam dump /data/app/bluefin/opt/sandshark/share/sandshark_apps/config/dynamiccontrol.yaml /dynamiccontrol");
    res.accepted = true;
  } else {
    res.accepted = false;
  }
  return true;
}

void DynamicControlApp::startupInitCallback() {
  _commandSub = _publicNode->subscribe("command", 1, &DynamicControlApp::msgCommandCallback, this);
  _navSub = _publicNode->subscribe("/navigation/navState", 1, &DynamicControlApp::msgNavigationCallback, this);
  _curTailSub = _publicNode->subscribe("/tailcone/currentpos", 1, &DynamicControlApp::msgCurrentTailconeCallback, this);

  _gainService = _publicNode->advertiseService("update_gain", &DynamicControlApp::handleGainUpdate, this);

  _tailCmdPub = _publicNode->advertise<sandshark_msgs::TailconeState>("tailcone_command", 10);
  _speedPub = _publicNode->advertise<sandshark_msgs::Speed>("estimated_speed", 10);
  _gainPub = _publicNode->advertise<sandshark_msgs::DynamicControlGains>("current_gains", 10);

  _depthPID = new PIDController("depth");
  _depthPID->initialize(_publicNode, _privateNode);

  _pitchPID = new PIDController("pitch");
  _pitchPID->initialize(_publicNode, _privateNode);

  _headingPID = new PIDController("heading");
  _headingPID->initialize(_publicNode, _privateNode);

  _privateNode->param("rpm_to_speed", _rpmToMetersPerSec, double(0.001));

  _privateNode->param("correctTailconePerRoll", _correctTailconePerRoll, false);

  //we can only read this in as a map of string to string
  _privateNode->getParam("rpm_to_speed_map", _speeds);

  //immediately convert it to a sorted vector of doubles to make interpolation easier
  for (std::map<std::string, std::string>::iterator it = _speeds.begin() ; it != _speeds.end(); ++it) {
    _speedV.push_back(std::pair<double, double>(atof(it->first.c_str()), atof(it->second.c_str())));
  }
  //TODO get this working with class member.  Probably need boost bind
  std::sort (_speedV.begin(), _speedV.end(), SpeedVCompare);

  _rate = new ros::Rate(20);
}

void DynamicControlApp::cleanup() {

}

bool DynamicControlApp::doInitialize() {
  cleanup();

  _lastRunTime = ros::Time::now();
  return true;
}

bool DynamicControlApp::doRun() {
  static int count = 0;

  //Publish the current gains before checking commands
  if (count >= 20) {
    _gainMsg.headingP = _headingPID->getPropGain();
    _gainMsg.headingI = _headingPID->getIntGain();
    _gainMsg.headingD = _headingPID->getDerGain();

    _gainMsg.depthP = _depthPID->getPropGain();
    _gainMsg.depthI = _depthPID->getIntGain();
    _gainMsg.depthD = _depthPID->getDerGain();

    _gainMsg.pitchP = _pitchPID->getPropGain();
    _gainMsg.pitchI = _pitchPID->getIntGain();
    _gainMsg.pitchD = _pitchPID->getDerGain();

    _gainMsg.rpm_to_speed = _rpmToMetersPerSec;

    _gainPub.publish(_gainMsg);
    count = 0;
  }
  count++;

  ros::Time now = ros::Time::now();
  if ((now - _lastCommandedTime) > ros::Duration(1.0)) {
    //Commands are old, do nothing
    return true;
  }

  if ((now - _lastMeasuredTime) > ros::Duration(0.5)) {
    //Measurements are old, do nothing
    return true;
  }

  ros::Duration dt = now - _lastRunTime;
  _lastRunTime = now;

  if (dt > ros::Duration(0.5)) {
    dt = ros::Duration(0.1);
    _depthPID->reset();
    _pitchPID->reset();
    _headingPID->reset();
  }

  double vertCmd = _verticalDesired;
  switch (_vertical_mode) {
  case (VM_INVALID_MIN):
  case (VM_INVALID_MAX):
    return true;
    break;
  case (VM_ALTITUDE):
    //essentially convert the altitude to a depth and pass it down
    vertCmd = _depthMeasured + _altitudeMeasured - vertCmd;
  case (VM_DEPTH):
    //Should be in radians?!?!?!?, output should be radians but input is meters
    vertCmd = -1.0 * _depthPID->process(vertCmd, _depthMeasured, dt.toSec());
  case (VM_PITCH):
    if (_vertical_mode == VM_PITCH) {
      //convert to radians if the pitch command is direct, otherwise it will already be radians
      vertCmd = ( _verticalDesired / 180.0 ) * M_PI;
    }
    vertCmd = _pitchPID->process(vertCmd, (_pitchMeasured / 180.0 * M_PI), dt.toSec());
    //here we are putting our pitch pid output radians into the elevator PID (in firmware)
    //we might need a conversion factor like the depth case, although it is less likely
  case (VM_ELEVATOR):
    if (_vertical_mode == VM_ELEVATOR) {
      //convert to radians if the pitch command is direct, otherwise it will already be radians
      vertCmd = ( _verticalDesired / 180.0 ) * M_PI;
    }
    // convert output from radians to degrees and send to the elevator
    _tailCmdMsg.elevator = (vertCmd / M_PI) * 180.0;
    break;
  }

  double horizCmd = _horizontalDesired / 180.0 * M_PI;
  switch (_horizontal_mode) {
  case (HM_INVALID_MIN):
  case (HM_INVALID_MAX):
    return true;
    break;
  case (HM_HEADING):
    horizCmd = _headingPID->process(horizCmd, (_bearingMeasured / 180.0 * M_PI), dt.toSec());
  case (HM_RUDDER):
    //Output rudder in degrees
    _tailCmdMsg.rudder = (horizCmd / M_PI) * 180.0;
    break;
  }

  if (_correctTailconePerRoll) {
    double rudder = _tailCmdMsg.rudder;
    double elevator = _tailCmdMsg.elevator;
    double rollRads = _rollMeasured / 180.0 * M_PI;

    _tailCmdMsg.elevator = elevator * cos(rollRads) + rudder * sin(rollRads);
    _tailCmdMsg.rudder = rudder * cos(rollRads) - elevator * sin(rollRads);
  }

  _tailCmdMsg.thruster = _rpmDesired;
  //Flip for reverse motion unless its a direct command
  if (_rpmDesired < 0.0) {
    if( _horizontal_mode != HM_RUDDER ) {
      _tailCmdMsg.rudder *= -1.0;
    }
    if( _vertical_mode != VM_ELEVATOR ) {
      _tailCmdMsg.elevator *= -1.0;
    }
  }
  _tailCmdPub.publish(_tailCmdMsg);

  return true;
}

double DynamicControlApp::getMPSfromRPM(double rpm) {
  double waterSpeed = 0.0;
  //linear interpolation
  for (std::vector<std::pair<double, double> >::iterator it = _speedV.begin(); it != _speedV.end(); ++it) {
    //go until we find the appropriate bin
    std::vector<std::pair<double, double> >::iterator nextIt = it;
    ++nextIt;
    if (rpm >= it->first && rpm < nextIt->first) {
      //check that we have a next point
      if (nextIt != _speedV.end()) {
        waterSpeed = it->second + (rpm - it->first) * ((nextIt->second - it->second) / ((nextIt->first - it->first)));
      } else {
        //end of the vector, use the max speed value, as the vehicle can't physically go faster
        waterSpeed = it->second;
      }
    }
  }
  return waterSpeed;
}

void DynamicControlApp::handleSleep() {
  _rate->sleep();
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::DynamicControlApp as;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) as, argc, argv);
}

