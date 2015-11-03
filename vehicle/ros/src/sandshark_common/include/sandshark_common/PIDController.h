#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>
#include <sandshark_msgs/PID.h>

namespace bluefin {
namespace sandshark {
class PIDController {
  private:
    bool _initialized;
    std::string _name;

    double _pgain, _igain, _dgain;
    double _imax;
    bool _cmdClip, _wrapAngles;
    double _cmdMin, _cmdMax;

    bool _hasPreviousMeasurement;
    double _prevMeasurement;

    ros::Publisher _pidPub;
    sandshark_msgs::PID _pidMsg;
  public:
    explicit PIDController(const char * pidName) :
        _initialized(false), _name(pidName) {
    }

    bool initialize(ros::NodeHandle *publicNode, ros::NodeHandle *privateNode);
    void setGains(double pgain, double igain, double dgain);
    void setIntegralLimits(double imax);
    void setOutputClipLimits(double min, double max);

    void updateNode(ros::NodeHandle *privateNode);

    double getPropGain() const {
      return _pgain;
    }
    ;
    double getIntGain() const {
      return _igain;
    }
    ;
    double getDerGain() const {
      return _dgain;
    }
    ;

    double process(double desired, double measured, double dt, bool hasRate = false, double measuredRate = 0.0);
    void reset();
};
}
}

#endif
