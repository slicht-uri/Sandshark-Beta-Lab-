#include <sandshark_common/task_base.h>
#include <sandshark_common/version.h>

namespace bluefin {
namespace sandshark {
void TaskBase::startupInitialization() {
  _publicNode = new ros::NodeHandle(_topicNS);
  _privateNode = new ros::NodeHandle("~");

  _state = TB_INIT;
  _appPub = _publicNode->advertise<sandshark_msgs::ApplicationStatus>("app_status", 1);
  _appMsg.appname = _taskName;
  _appMsg.pid = getpid();
  _appMsg.status = _state;
  _appMsg.error = "";
  _appMsg.version = VERSION;
  _appMsg.heartbeat = true;
  _appPub.publish(_appMsg);

  _privateNode->param("logLocal", _logLocal, false);
  _privateNode->param("logPrefix", _logPrefix, std::string("/mnt/sdcard/"));
  _privateNode->param("startDelay", _startDelay, double(0.0));

  _logFile = NULL;
  if (_logLocal) {
    _logFile = fopen((_logPrefix + _topicNS + timeToStr(ros::WallTime::now()) + std::string(".log")).c_str(), "w");
  }

  _appRestartSub = _publicNode->subscribe<sandshark_msgs::ApplicationRestartCommand>(
      std::string("/health/control/") + _taskName, 1, &TaskBase::msgAppRestartCallback, this);

  _initTime = ros::Time::now();
  _lastAppPubTime = ros::Time::now();
  startupInitCallback();
}

void TaskBase::msgAppRestartCallback(const sandshark_msgs::ApplicationRestartCommand::ConstPtr & msg) {
  _attemptRestart = true;
}

void TaskBase::run() {

  //only start after the delay is over
  ros::Time now = ros::Time::now();

  switch (_state) {
  case (TB_INIT):
    if (doInitialize()) {
      if (now - _initTime < ros::Duration(_startDelay)) {
        break;
      }
      ROS_INFO("Task Init - switching to run!\n");
      _state = TB_RUNNING;
    } else {
      _state = TB_ERROR;
    }
    _shouldCleanup = true;
    break;
  case (TB_RUNNING):
    if (!doRun()) {
      _state = TB_ERROR;
    }
    break;
  case (TB_ERROR):
    ROS_DEBUG_ONCE("Entering Error State!");
    if (_shouldCleanup) {
      cleanup();
      _shouldCleanup = false;
    }

    if (_attemptRestart) {
      ROS_INFO("Attempting Restart!");
      _state = TB_INIT;
    }
    break;
  }

  _attemptRestart = false;
  now = ros::Time::now();
  if ((now - _lastAppPubTime) > ros::Duration(1.0)) {
    _appMsg.heartbeat = !_appMsg.heartbeat;
    _appMsg.status = _state;
    _appPub.publish(_appMsg);
    _lastAppPubTime = now;
  }
}

template<class T>
std::string TaskBase::timeToStr(T ros_t) {
  std::stringstream msg;
  const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
  boost::posix_time::time_facet * const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
  msg.imbue(std::locale(msg.getloc(), f));
  msg << now;
  return msg.str();
}

}
}
