#ifndef _BLUEFIN__SANDSHARK__SANDSHARK_COMMON__TASK_BASE_H_
#define _BLUEFIN__SANDSHARK__SANDSHARK_COMMON__TASK_BASE_H_

#include <ros/ros.h>
#include <string>
#include <sandshark_msgs/ApplicationStatus.h>
#include <sandshark_msgs/ApplicationRestartCommand.h>
#include <boost/date_time/local_time/local_time.hpp>

namespace bluefin {
namespace sandshark {

enum DynamicControlVerticalMode {
  VM_INVALID_MIN = -1,
  VM_DEPTH = 0,
  VM_ALTITUDE = 1,
  VM_PITCH = 2,
  VM_ELEVATOR = 3,
  VM_INVALID_MAX,
};

enum DynamicControlHorizontalMode {
  HM_INVALID_MIN = -1,
  HM_HEADING = 0,
  HM_RUDDER = 1,
  HM_INVALID_MAX,
};

enum CustomObjective {
  ADAPT_GPS_SURFACE = 1,
};

enum DynamicControlGainUpdate {
  GU_INVALID_MIN = -1,
  GU_HEADING_P = 0,
  GU_HEADING_I = 1,
  GU_HEADING_D = 2,
  GU_DEPTH_P = 3,
  GU_DEPTH_I = 4,
  GU_DEPTH_D = 5,
  GU_PITCH_P = 6,
  GU_PITCH_I = 7,
  GU_PITCH_D = 8,
  GU_RPMTOSPEED = 9,
  GU_FLUOR = 10,
  GU_INVALID_MAX,
};

enum ObjectiveState {
  INVALID_OBJECTIVE = -1,
};

enum HealthValueUpdate {
  HV_INVALID_MIN = -1,
  HV_MAX_TEMP = 0,
  HV_MAX_PRESSURE = 1,
  HV_MIN_PRESSURE = 2,
  HV_MAX_DEPTH = 3,
  HV_MIN_ALTITUDE = 4,
  HV_MIN_BATTERY = 5,
  HV_MAX_MISSION_SECONDS = 6,
  HV_INVALID_MAX,
};

class TaskBase {
  public:
    enum TaskBaseState {
      TB_INIT, TB_RUNNING, TB_ERROR
    };
  private:
    std::string _taskName;
    std::string _topicNS;

    ros::Subscriber _appRestartSub;
    ros::Publisher _appPub;
    sandshark_msgs::ApplicationStatus _appMsg;
    ros::Time _lastAppPubTime;

    bool _attemptRestart;
    bool _shouldCleanup;

    std::string _logPrefix;
  protected:
    enum TaskBaseState _state;
    ros::NodeHandle *_publicNode;
    ros::NodeHandle *_privateNode;

    explicit TaskBase(std::string taskName, std::string topic_ns) :
        _taskName(taskName), _topicNS(topic_ns), _publicNode( NULL), _privateNode( NULL) {
    }

    enum TaskBaseState getState() {
      return _state;
    }

    void setErrorMessage(std::string err) {
      _appMsg.error = err;
    }

    void clearErrorMessage() {
      _appMsg.error = "";
    }

    virtual void startupInitCallback() {
    }
    virtual bool doInitialize() = 0;
    virtual bool doRun() = 0;
    virtual void cleanup() = 0;

    struct NavState {
        double latitude;
        double longitude;
        double speed;
        double depth;
        double altitude;
        double pitch;
        double roll;
        double bearing;
        long long int epoch;
        bool isValid;
        ros::Time receivedTime;
        int quality;
        ros::Time computed_time;
    };

    template<class T> std::string timeToStr(T ros_t);
    bool _logLocal;
    FILE *_logFile;
    double _startDelay;
    ros::Time _initTime;
  public:
    std::string & getTaskName() {
      return _taskName;
    }

    void msgAppRestartCallback(const sandshark_msgs::ApplicationRestartCommand::ConstPtr & msg);

    virtual void startupInitialization();

    virtual void run();

    virtual void handleSleep() = 0;
    virtual void handleShutdown() {
    }

    virtual ~TaskBase() {
    }
};

}
}

#endif

