#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/condition.h>

#include <Eigen>
#include <sys/syscall.h>

#include <ros/ros.h>
#include <sandshark_msgs/ApplicationAbort.h>
#include <sandshark_msgs/ApplicationStatus.h>
#include <sandshark_msgs/ApplicationHealthStatus.h>
#include <sandshark_msgs/BoardHealthStatus.h>
#include <sandshark_msgs/Environmental.h>
#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/LastAbort.h>
#include <sandshark_msgs/LeakDetect.h>
#include <sandshark_msgs/GasGauge.h>
#include <sandshark_msgs/ApplicationRestartCommand.h>
#include <sandshark_msgs/ElapsedMissionTime.h>
#include <sandshark_msgs/HealthValues.h>
#include <sandshark_msgs/UpdateHealthVal.h>

using namespace std;
using namespace Eigen;
namespace bluefin {
namespace sandshark {

template<typename CompareOp>
class TemperatureCondition: public ConditionTemplate<sandshark_msgs::GasGauge, double, CompareOp> {
  private:
    std::string reason;
    virtual void handleMessage() {

      this->_isTriggered = this->_operation(this->_msg->temperature, this->_compareVal);
      ROS_INFO("Testing Temp Condition temp is %f, triggered is %s", this->_msg->temperature,
          (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        char temp[128];
        snprintf(&temp[0], sizeof(temp), "Temperature of %f is %s %f", this->_msg->temperature,
            this->_opDescription.c_str(), this->_compareVal);
        temp[sizeof(temp) - 1] = '\0';
        reason = std::string(temp);
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit TemperatureCondition(CompareOp op, double val, std::string desc, ros::NodeHandle * r) :
        ConditionTemplate<sandshark_msgs::GasGauge, double, CompareOp>(op, val, desc) {
      this->_topicName = "/gasgauge/gasgauge";
      this->_prNode = r;
    }
};

template<typename CompareOp>
class PressureCondition: public ConditionTemplate<sandshark_msgs::Environmental, double, CompareOp> {
  private:
    std::string reason;
    virtual void handleMessage() {

      this->_isTriggered = this->_operation(this->_msg->pressure, this->_compareVal);
      ROS_INFO("Testing Pressure = %f, triggered is %s", this->_msg->pressure, (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        char temp[128];
        snprintf(&temp[0], sizeof(temp), "Pressure of %f is %s %f", this->_msg->pressure, this->_opDescription.c_str(),
            this->_compareVal);
        temp[sizeof(temp) - 1] = '\0';
        reason = std::string(temp);
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit PressureCondition(CompareOp op, double val, std::string desc, ros::NodeHandle * r) :
        ConditionTemplate<sandshark_msgs::Environmental, double, CompareOp>(op, val, desc) {
      this->_topicName = "/environmental/environmental";
    }
};

template<typename CompareOp>
class DepthCondition: public ConditionTemplate<sandshark_msgs::Navigation, double, CompareOp> {
  private:
    std::string reason;
    virtual void handleMessage() {

      this->_isTriggered = this->_operation(this->_msg->depth, this->_compareVal);
      ROS_INFO("Testing Depth = %f, triggered is %s", this->_msg->depth, (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        char temp[128];
        snprintf(&temp[0], sizeof(temp), "Depth of %f is %s %f", this->_msg->depth, this->_opDescription.c_str(),
            this->_compareVal);
        temp[sizeof(temp) - 1] = '\0';
        reason = std::string(temp);
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit DepthCondition(CompareOp op, double val, std::string desc, ros::NodeHandle * r) :
        ConditionTemplate<sandshark_msgs::Navigation, double, CompareOp>(op, val, desc) {
      this->_topicName = "/navigation/navState";
    }
};

template<typename CompareOp>
class AltitudeCondition: public ConditionTemplate<sandshark_msgs::Navigation, double, CompareOp> {
  private:
    std::string reason;
    virtual void handleMessage() {

      //Do we need consecutive checks here? - probably not now that we filter
      this->_isTriggered = (this->_msg->altitude > 0.0) && this->_operation(this->_msg->altitude, this->_compareVal);
      ROS_INFO("Testing Altitude = %f, triggered is %s", this->_msg->altitude, (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        char temp[128];
        snprintf(&temp[0], sizeof(temp), "Altitude of %f is %s %f", this->_msg->altitude, this->_opDescription.c_str(),
            this->_compareVal);
        temp[sizeof(temp) - 1] = '\0';
        reason = std::string(temp);
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit AltitudeCondition(CompareOp op, double val, std::string desc, ros::NodeHandle * r) :
        ConditionTemplate<sandshark_msgs::Navigation, double, CompareOp>(op, val, desc) {
      this->_topicName = "/navigation/navState";
    }
};

template<typename CompareOp>
class BatteryCondition: public ConditionTemplate<sandshark_msgs::GasGauge, double, CompareOp> {
  private:
    std::string reason;
    virtual void handleMessage() {

      this->_isTriggered = this->_operation(this->_msg->state_of_charge, this->_compareVal);
      ROS_INFO("Testing Battery Condition percent state of charge is %f, triggered is %s", this->_msg->state_of_charge,
          (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        char psoc[128];
        snprintf(&psoc[0], sizeof(psoc), "Battery PSOC of %f is %s %f", this->_msg->state_of_charge,
            this->_opDescription.c_str(), this->_compareVal);
        psoc[sizeof(psoc) - 1] = '\0';
        reason = std::string(psoc);
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit BatteryCondition(CompareOp op, double val, std::string desc, ros::NodeHandle * r) :
        ConditionTemplate<sandshark_msgs::GasGauge, double, CompareOp>(op, val, desc) {
      this->_topicName = "/gasgauge/gasgauge";
    }
};

template<typename CompareOp>
class MissionTimeCondition: public ConditionTemplate<sandshark_msgs::ElapsedMissionTime, double, CompareOp> {
  private:
    std::string reason;
    virtual void handleMessage() {

      this->_isTriggered = this->_operation(this->_msg->seconds_elapsed, this->_compareVal);
      ROS_INFO("Testing mission elapsed seconds is %f, triggered is %s", this->_msg->seconds_elapsed,
          (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        char s[128];
        snprintf(&s[0], sizeof(s), "elapsed seconds of %f is %s %f", this->_msg->seconds_elapsed,
            this->_opDescription.c_str(), this->_compareVal);
        s[sizeof(s) - 1] = '\0';
        reason = std::string(s);
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit MissionTimeCondition(CompareOp op, double val, std::string desc, ros::NodeHandle * r) :
        ConditionTemplate<sandshark_msgs::ElapsedMissionTime, double, CompareOp>(op, val, desc) {
      this->_topicName = "/objectivecontrol/elapsed_time";
    }
};

class LeakCondition: public ConditionTemplate<sandshark_msgs::LeakDetect, bool, std::equal_to<bool> > {
  private:
    std::string reason;
    virtual void handleMessage() {
      //Do we need consecutive checks here?
      this->_isTriggered = this->_operation(this->_msg->leak, this->_compareVal);
      if (this->_isTriggered) {
        reason = std::string("Leak detect is positive!");
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit LeakCondition(std::string desc) :
        ConditionTemplate<sandshark_msgs::LeakDetect, bool, std::equal_to<bool> >(std::equal_to<bool>(), true, desc) {
      this->_topicName = "/currentsense/leak_detect";
    }
};

class NavLossCondition: public ConditionTemplate<sandshark_msgs::Navigation, bool, std::equal_to<bool> > {
  private:
    std::string reason;
    virtual void handleMessage() {
      //Do we need consecutive checks here?
      this->_isTriggered = this->_operation(this->_msg->is_valid, this->_compareVal);
      ROS_INFO("Testing invalid nav = %s, triggered is %s", (this->_msg->is_valid ? "true" : "false"), (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        reason = this->_msg->invalid_reason;
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit NavLossCondition(std::string desc) :
        ConditionTemplate<sandshark_msgs::Navigation, bool, std::equal_to<bool> >(std::equal_to<bool>(), false, desc) {
      this->_topicName = "/navigation/navState";
    }
};

class EthernetCondition: public ConditionTemplate<sandshark_msgs::BoardHealthStatus, bool, std::equal_to<bool> > {
  private:
    std::string reason;
    virtual void handleMessage() {
      this->_isTriggered = this->_operation(this->_msg->good_eth, this->_compareVal);
      ROS_INFO("Testing eth0 = %d, triggered is %s", this->_msg->good_eth, (this->_isTriggered ? "true" : "false"));
      if (this->_isTriggered) {
        char temp[128];
        snprintf(&temp[0], sizeof(temp), "Needed ethernet interface (eth0) does not exist!");
        temp[sizeof(temp) - 1] = '\0';
        reason = std::string(temp);
      } else {
        reason = "";
      }
    }
  public:
    std::string getReason() {
      return reason;
    }
    explicit EthernetCondition(std::string desc) :
        ConditionTemplate<sandshark_msgs::BoardHealthStatus, bool, std::equal_to<bool> >(std::equal_to<bool>(), false,
            desc) {
      this->_topicName = "/health/board_status";
    }
};

class HealthApp: public TaskBase {
  private:
    enum {
      PERCENT_USAGE_FILTER = 10
    };

    struct pstat {
        unsigned char state;
        long unsigned int utime_ticks;
        long int cutime_ticks;
        long unsigned int stime_ticks;
        long int cstime_ticks;
        int priority;
        unsigned int num_threads;
        long unsigned int vsize; // virtual memory size in bytes
        long unsigned int rss; //Resident  Set  Size in bytes
        long unsigned int cpu_total_time;
        int num_restarts;
        double firstRestart;
        double percentUsage[PERCENT_USAGE_FILTER];
        double timestamp;
    };

    map<string, ros::Subscriber> _appSubMap;
    map<string, int> _topicToPID;
    map<int, struct pstat> _pidToStat;
    map<int, ros::Publisher> _pidToPublisher;
    map<string, ros::Publisher> _topicToRestartPublisher;
    ros::Publisher _boardHealthPub;

    sandshark_msgs::ApplicationHealthStatus _appHealthMsg;
    sandshark_msgs::BoardHealthStatus _boardHealthMsg;
    sandshark_msgs::ApplicationRestartCommand _appRestartMsg;

    ros::ServiceServer _valueService;

    ros::Publisher _lastAbortPub;
    sandshark_msgs::LastAbort _lastAbortMsg;
    ros::Subscriber _applicationAbortSub;

    ros::Publisher _valuePub;
    sandshark_msgs::HealthValues _valueMsg;

    ros::Rate *_rate;

    TemperatureCondition<std::greater<double> > * _tempCondition;
    PressureCondition<std::greater<double> > * _maxPressCondition;
    PressureCondition<std::less<double> > * _minPressCondition;
    DepthCondition<std::greater<double> > * _depthCondition;
    AltitudeCondition<std::less<double> > * _altCondition;
    BatteryCondition<std::less<double> > * _batteryCondition;
    MissionTimeCondition<std::greater<double> > * _missionTimeCondition;

    int _updateParamDivisor;

  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
    void cleanup();

    void publishBoardHealth();
    int get_usage(const pid_t pid, struct pstat* result);
    void calc_cpu_usage_pct(const struct pstat* cur_usage, const struct pstat* last_usage, double* usage);
  public:
    HealthApp() :
        TaskBase("Health", "health") {
    }

    void msgApplicationAbortCallback(const sandshark_msgs::ApplicationAbort::ConstPtr & msg);
    void msgAppStateCallback(const sandshark_msgs::ApplicationStatus::ConstPtr & msg, const string & topic);

    void handleSleep();
    bool handleValueUpdate(sandshark_msgs::UpdateHealthVal::Request &req,
        sandshark_msgs::UpdateHealthVal::Response &res);
};

TaskBase *getTaskBase() {
  return new HealthApp();
}

void HealthApp::startupInitCallback() {
  int pid = getpid();
  int affinity = 1;
  int sysret = syscall( __NR_sched_setaffinity, pid, sizeof(affinity), &affinity);
  cout << "Health set affinity- PID: " << pid << " sysret: " << sysret << " affinity: " << affinity << endl;

  ConditionManager::instance()->updateAbortAction(_publicNode);
  ConditionManager::instance()->updateSolidAllLEDAction(_publicNode);

  double max_temp;
  _privateNode->param("max_temperature", max_temp, (double) 35.0);
  _tempCondition = new TemperatureCondition<std::greater<double> >(std::greater<double>(), max_temp, "greater than",
      _privateNode);
  Condition * tempCondition = (Condition *) _tempCondition;
  ConditionManager::instance()->addCondition(_publicNode, tempCondition);
  ConditionManager::instance()->linkConditionToAction(tempCondition, ConditionManager::instance()->getAbortAction());

  double max_pressure;
  _privateNode->param("max_pressure", max_pressure, (double) 90.0);
  _maxPressCondition = new PressureCondition<std::greater<double> >(std::greater<double>(), max_pressure, "greater than",
      _privateNode);
  Condition * maxPressCondition = (Condition *) _maxPressCondition;
  ConditionManager::instance()->addCondition(_publicNode, maxPressCondition);
  ConditionManager::instance()->linkConditionToAction(maxPressCondition, ConditionManager::instance()->getAbortAction());

  double min_pressure;
  _privateNode->param("min_pressure", min_pressure, (double) 105.0);
  _minPressCondition = new PressureCondition<std::less<double> >(std::less<double>(), min_pressure, "less than",
      _privateNode);
  Condition * minPressCondition = (Condition *) _minPressCondition;
  ConditionManager::instance()->addCondition(_publicNode, minPressCondition);
  ConditionManager::instance()->linkConditionToAction(minPressCondition, ConditionManager::instance()->getAbortAction());

  double max_depth;
  _privateNode->param("max_depth", max_depth, (double) 100.0);
  _depthCondition = new DepthCondition<std::greater<double> >(std::greater<double>(), max_depth, "greater than",
      _privateNode);
  Condition * depthCondition = (Condition *) _depthCondition;
  ConditionManager::instance()->addCondition(_publicNode, depthCondition);
  ConditionManager::instance()->linkConditionToAction(depthCondition, ConditionManager::instance()->getAbortAction());

  double max_altitude;
  _privateNode->param("min_altitude", max_altitude, (double) 1.0);
  _altCondition = new AltitudeCondition<std::less<double> >(std::less<double>(), max_altitude, "less than",
      _privateNode);
  Condition * altCondition = (Condition *) _altCondition;
  ConditionManager::instance()->addCondition(_publicNode, altCondition);
  ConditionManager::instance()->linkConditionToAction(altCondition, ConditionManager::instance()->getAbortAction());

  double min_battery;
  _privateNode->param("min_battery", min_battery, (double) 5.0);
  _batteryCondition = new BatteryCondition<std::less<double> >(std::less<double>(), min_battery, "less than",
      _privateNode);
  Condition * batteryCondition = (Condition *) _batteryCondition;
  ConditionManager::instance()->addCondition(_publicNode, batteryCondition);
  ConditionManager::instance()->linkConditionToAction(batteryCondition, ConditionManager::instance()->getAbortAction());

  double max_mission_seconds;
  _privateNode->param("max_mission_seconds", max_mission_seconds, (double) 3600.0);
  _missionTimeCondition = new MissionTimeCondition<std::greater<double> >(std::greater<double>(), max_mission_seconds,
      "greater than", _privateNode);
  Condition * missionTimeCondition = (Condition *) _missionTimeCondition;
  ConditionManager::instance()->addCondition(_publicNode, missionTimeCondition);
  ConditionManager::instance()->linkConditionToAction(missionTimeCondition,
      ConditionManager::instance()->getAbortAction());

  Condition *leakCondition = new LeakCondition("leaking");
  ConditionManager::instance()->addCondition(_publicNode, leakCondition);
  ConditionManager::instance()->linkConditionToAction(leakCondition, ConditionManager::instance()->getAbortAction());

  Condition *navLossCondition = new NavLossCondition("nav_loss");
  ConditionManager::instance()->addCondition(_publicNode, navLossCondition);
  ConditionManager::instance()->linkConditionToAction(navLossCondition, ConditionManager::instance()->getAbortAction());

  Condition *ethCondition = new EthernetCondition("exists");
  ConditionManager::instance()->addCondition(_publicNode, ethCondition);
  ConditionManager::instance()->linkConditionToAction(ethCondition, ConditionManager::instance()->getAbortAction());
  ConditionManager::instance()->linkConditionToAction(ethCondition,
      ConditionManager::instance()->getSolidAllLEDAction());

  _boardHealthPub = _publicNode->advertise<sandshark_msgs::BoardHealthStatus>("board_status", 10);

  _updateParamDivisor = 0;

  _valueService = _publicNode->advertiseService("update_value", &HealthApp::handleValueUpdate, this);

  _valuePub = _publicNode->advertise<sandshark_msgs::HealthValues>("current_values", 10);

  _lastAbortPub = _publicNode->advertise<sandshark_msgs::LastAbort>("last_abort", 1);
  _applicationAbortSub = _publicNode->subscribe("/objectivecontrol/abort", 1, &HealthApp::msgApplicationAbortCallback, this);

  system("rosparam load /data/app/bluefin/opt/sandshark/share/sandshark_apps/config/health.yaml /health");

}

bool HealthApp::doInitialize() {
  _rate = new ros::Rate(1.0);
  return true;
}

void HealthApp::msgApplicationAbortCallback(const sandshark_msgs::ApplicationAbort::ConstPtr & msg) {
  _lastAbortMsg.abort_reason = msg->reason;
  _lastAbortMsg.abort_time = ros::Time::now();
}

void HealthApp::msgAppStateCallback(const sandshark_msgs::ApplicationStatus::ConstPtr & msg, const string & topic) {
  map<string, int>::iterator iter = _topicToPID.find(topic);
  if (iter == _topicToPID.end()) {
    ROS_INFO("Creating new stat for pid %d, name %s", msg->pid, msg->appname.c_str());
    _topicToPID[topic] = msg->pid;
    get_usage(msg->pid, &(_pidToStat[msg->pid]));
    for (int i = 0; i < PERCENT_USAGE_FILTER; ++i) {
      _pidToStat[msg->pid].percentUsage[i] = 0.0;
    }
    _pidToStat[msg->pid].num_restarts = 0;
    _pidToPublisher[msg->pid] = _publicNode->advertise<sandshark_msgs::ApplicationHealthStatus>(
        std::string("stats/") + msg->appname, 10);
  } else {
    if (_topicToPID[topic] != (int) msg->pid) {
      ROS_WARN("received message from topic %s from 2 pids, orig: %d new: %d", topic.c_str(), _topicToPID[topic],
          msg->pid);
      return;
    }

    struct pstat curr_stat;
    double pct;
    memcpy(&curr_stat, &(_pidToStat[msg->pid]), sizeof(struct pstat));
    get_usage(msg->pid, &curr_stat);
    calc_cpu_usage_pct(&curr_stat, &(_pidToStat[msg->pid]), &pct);
    memcpy(&(_pidToStat[msg->pid]), &curr_stat, sizeof(struct pstat));

    memcpy(&(_pidToStat[msg->pid].percentUsage[0]), &(_pidToStat[msg->pid].percentUsage[1]),
        sizeof(double) * (PERCENT_USAGE_FILTER - 1));
    _pidToStat[msg->pid].percentUsage[PERCENT_USAGE_FILTER - 1] = pct;
  }

  if (_pidToStat[msg->pid].num_restarts > 0
      && ((ros::WallTime::now().toSec() - _pidToStat[msg->pid].firstRestart) > 10.0)) {
    _pidToStat[msg->pid].num_restarts = 0;
  }

  map<string, ros::Publisher>::iterator tPubIter = _topicToRestartPublisher.find(topic);
  if (tPubIter == _topicToRestartPublisher.end()) {
    _topicToRestartPublisher[topic] = _publicNode->advertise<sandshark_msgs::ApplicationRestartCommand>(
        std::string("control/") + msg->appname, 10);
  } else {
    _appRestartMsg.pid = msg->pid;
    _appRestartMsg.restart = false;
    _appRestartMsg.heartbeat = msg->heartbeat;
    if (msg->status == TaskBase::TB_ERROR) {
      ROS_INFO("Attempting to restart app %d:%s num restarts = %d", msg->pid, msg->appname.c_str(),
          _pidToStat[msg->pid].num_restarts);
      if (_pidToStat[msg->pid].num_restarts == 0) {
        _pidToStat[msg->pid].firstRestart = ros::WallTime::now().toSec();
      }
      _pidToStat[msg->pid].num_restarts++;
      if (_pidToStat[msg->pid].num_restarts > 5) {
        ConditionManager::instance()->getAbortAction()->execute(
            msg->appname + std::string("has had too many restarts"));
      }
      _appRestartMsg.restart = true;
    }
    tPubIter->second.publish(_appRestartMsg);
  }

}

void HealthApp::cleanup() {
  //nothing to do...
}

bool HealthApp::doRun() {

  static int count = 0;
  if (count > 100)
    count = 0;

  XmlRpc::XmlRpcValue params("ros_topic_list");
  XmlRpc::XmlRpcValue results;
  XmlRpc::XmlRpcValue r;

  //subscribe to all apps app_status message
  if (ros::master::execute("getTopicTypes", params, results, r, false) == true) {
    if (results.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      int32_t i = 2;
      if (results[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int32_t j = 0; j < results[i].size(); ++j) {
          if (results[i][j].getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (results[i][j].size() == 2) {
              if (results[i][j][0].getType() == XmlRpc::XmlRpcValue::TypeString
                  && results[i][j][1].getType() == XmlRpc::XmlRpcValue::TypeString) {
                std::string topic = static_cast<std::string>(results[i][j][0]);
                std::string type = static_cast<std::string>(results[i][j][1]);
                //std::cout<<"Topic : "<<topic<<" -> "<<type<<std::endl;
                if (type == "sandshark_msgs/ApplicationStatus") {
                  map<string, ros::Subscriber>::iterator iter = _appSubMap.find(topic);
                  if (iter == _appSubMap.end()) {
                    _appSubMap[topic] = _publicNode->subscribe<sandshark_msgs::ApplicationStatus>(topic, 1,
                        boost::bind(&HealthApp::msgAppStateCallback, this, _1, topic));
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  //publish vehicle health
  for (map<int, struct pstat>::iterator iter = _pidToStat.begin(); iter != _pidToStat.end(); ++iter) {
    _appHealthMsg.pid = iter->first;
    double cpu_usage = 0.0;
    for (int i = 0; i < PERCENT_USAGE_FILTER; ++i) {
      cpu_usage += iter->second.percentUsage[i];
    }
    cpu_usage /= (double) PERCENT_USAGE_FILTER;
    _appHealthMsg.cpu_usage = cpu_usage;
    _appHealthMsg.virtual_mem_size = iter->second.vsize;
    _appHealthMsg.resident_mem_size = iter->second.rss;
    _appHealthMsg.state = iter->second.state;
    _appHealthMsg.num_threads = iter->second.num_threads;
    _appHealthMsg.priority = iter->second.priority;

    _pidToPublisher[iter->first].publish(_appHealthMsg);
  }

  if (count % 10 == 0) {
    publishBoardHealth();
  }

  if (count % 20 == 0) {
    _valueMsg.max_temp = _tempCondition->getCompareVal();
    _valueMsg.max_pressure = _maxPressCondition->getCompareVal();
    _valueMsg.min_pressure = _minPressCondition->getCompareVal();
    _valueMsg.max_depth = _depthCondition->getCompareVal();
    _valueMsg.max_altitude = _altCondition->getCompareVal();
    _valueMsg.min_battery = _batteryCondition->getCompareVal();
    _valueMsg.max_mission_seconds = _missionTimeCondition->getCompareVal();

    _valuePub.publish(_valueMsg);
  }

  // we have to manually check every once in a while if parameters have changed
  double updatedParam;
  switch (_updateParamDivisor) {
  case (0):
    _publicNode->getParamCached("/health/max_temperature", updatedParam);
    _tempCondition->setCompareVal(updatedParam);
    _updateParamDivisor++;
    break;
  case (1):
    _publicNode->getParamCached("/health/max_pressure", updatedParam);
    _maxPressCondition->setCompareVal(updatedParam);
    _updateParamDivisor++;
    break;
  case (2):
    _publicNode->getParamCached("/health/min_pressure", updatedParam);
    _minPressCondition->setCompareVal(updatedParam);
    _updateParamDivisor++;
    break;
  case (3):
    _publicNode->getParamCached("/health/max_depth", updatedParam);
    _depthCondition->setCompareVal(updatedParam);
    _updateParamDivisor++;
    break;
  case (4):
    _publicNode->getParamCached("/health/min_altitude", updatedParam);
    _altCondition->setCompareVal(updatedParam);
    _updateParamDivisor++;
    break;
  case (5):
    _publicNode->getParamCached("/health/min_battery", updatedParam);
    _batteryCondition->setCompareVal(updatedParam);
    _updateParamDivisor++;
    break;
  case (6):
    _publicNode->getParamCached("/health/max_mission_seconds", updatedParam);
    _missionTimeCondition->setCompareVal(updatedParam);
    _updateParamDivisor = 0;
    break;
  }
  ConditionManager::instance()->executeTriggeredConditions();

  count++;

  _lastAbortPub.publish(_lastAbortMsg);

  return true;
}

void HealthApp::publishBoardHealth() {
  //find eth0
  std::ifstream f("/proc/net/dev");
  std::string line;
  bool foundEth0 = false;
  if (f) {
    while (getline(f, line)) {
      if (line.find("eth0: ", 0) != std::string::npos) {
        foundEth0 = true;
        break;
      }
    }
  } else {
    ROS_WARN("Cannot open /proc/net/dev. Could not check for interface eth0!");
  }

  _boardHealthMsg.good_eth = foundEth0;
  _boardHealthPub.publish(_boardHealthMsg);
}

void HealthApp::handleSleep() {
  _rate->sleep();
}

bool HealthApp::handleValueUpdate(sandshark_msgs::UpdateHealthVal::Request &req,
    sandshark_msgs::UpdateHealthVal::Response &res) {
  if ((req.updateValueID > HV_INVALID_MIN) && (req.updateValueID < HV_INVALID_MAX)) {
    enum HealthValueUpdate hvID = static_cast<enum HealthValueUpdate>(req.updateValueID);
    switch (hvID) {
    case (HV_MAX_TEMP):
      _privateNode->setParam("/health/max_temperature", req.value);
      _tempCondition->setCompareVal(req.value);
      break;

    case (HV_MAX_PRESSURE):
      _privateNode->setParam("/health/max_pressure", req.value);
      _maxPressCondition->setCompareVal(req.value);
      break;

    case (HV_MIN_PRESSURE):
      _privateNode->setParam("/health/min_pressure", req.value);
      _minPressCondition->setCompareVal(req.value);
      break;

    case (HV_MAX_DEPTH):
      _privateNode->setParam("/health/max_depth", req.value);
      _depthCondition->setCompareVal(req.value);
      break;

    case (HV_MIN_ALTITUDE):
      _privateNode->setParam("/health/min_altitude", req.value);
      _altCondition->setCompareVal(req.value);
      break;

    case (HV_MIN_BATTERY):
      _privateNode->setParam("/health/min_battery", req.value);
      _batteryCondition->setCompareVal(req.value);
      break;

    case (HV_MAX_MISSION_SECONDS):
      _privateNode->setParam("/health/max_mission_seconds", req.value);
      _missionTimeCondition->setCompareVal(req.value);
      break;

    case (HV_INVALID_MIN):
    case (HV_INVALID_MAX):
      //we shouldn't be here
      break;
    }
    system("rosparam dump /data/app/bluefin/opt/sandshark/share/sandshark_apps/config/health.yaml /health");
    res.accepted = true;
  } else {
    res.accepted = false;
  }
  return true;
}

int HealthApp::get_usage(const pid_t pid, struct pstat* result) {
//convert  pid to string
  static char stat_path[128];

  snprintf(&stat_path[0], sizeof(stat_path), "/proc/%d/stat", pid);
  stat_path[sizeof(stat_path) - 1] = '\0';

  FILE *fpstat = fopen(stat_path, "r");
  if (fpstat == NULL) {
    perror("FOPEN ERROR ");
    return -1;
  }

  FILE *fstat = fopen("/proc/stat", "r");
  if (fstat == NULL) {
    perror("FOPEN ERROR ");
    fclose(fstat);
    return -1;
  }

  //read values from /proc/pid/stat
  bzero(result, sizeof(struct pstat));
  long unsigned int rss;
  if (fscanf(fpstat, "%*d %*s %c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %lu"
      "%lu %ld %ld %d %*d %d %*d %*u %lu %ld", &result->state, &result->utime_ticks, &result->stime_ticks,
      &result->cutime_ticks, &result->cstime_ticks, &result->priority, &result->num_threads, &result->vsize,
      &rss) == EOF) {
    fclose(fpstat);
    return -1;
  }
  fclose(fpstat);
  result->rss = rss * getpagesize();

  //read+calc cpu total time from /proc/stat
  long unsigned int cpu_time[10];
  bzero(cpu_time, sizeof(cpu_time));
  if (fscanf(fstat, "%*s %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu", &cpu_time[0], &cpu_time[1], &cpu_time[2],
      &cpu_time[3], &cpu_time[4], &cpu_time[5], &cpu_time[6], &cpu_time[7], &cpu_time[8], &cpu_time[9]) == EOF) {
    fclose(fstat);
    return -1;
  }

  fclose(fstat);

  for (int i = 0; i < 4; i++)
    result->cpu_total_time += cpu_time[i];

  result->timestamp = ros::WallTime::now().toSec();
  return 0;
}

void HealthApp::calc_cpu_usage_pct(const struct pstat* cur_usage, const struct pstat* last_usage, double* usage) {
  const long unsigned int cpu_diff = cur_usage->cpu_total_time - last_usage->cpu_total_time;
  const long unsigned int pid_diff = (cur_usage->utime_ticks + cur_usage->utime_ticks + cur_usage->stime_ticks
      - cur_usage->stime_ticks)
      - (last_usage->utime_ticks + last_usage->utime_ticks + last_usage->stime_ticks - last_usage->stime_ticks);

  *usage = (1.0 / (cur_usage->timestamp - last_usage->timestamp)) * ((double) pid_diff / (double) cpu_diff) * 100.0;
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::HealthApp h;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) h, argc, argv);
}

