#include "sandshark_common/PIDController.h"

namespace bluefin {
namespace sandshark {

bool PIDController::initialize(ros::NodeHandle *publicNode, ros::NodeHandle *privateNode) {
  _pidPub = publicNode->advertise<sandshark_msgs::PID>(_name + std::string("PID"), 10);

  privateNode->param(_name + std::string("pgain"), _pgain, double(0.0));
  privateNode->param(_name + std::string("igain"), _igain, double(0.0));
  privateNode->param(_name + std::string("dgain"), _dgain, double(0.0));

  privateNode->param(_name + std::string("imax"), _imax, double(0.0));

  privateNode->param(_name + std::string("clipCmd"), _cmdClip, bool(false));
  privateNode->param(_name + std::string("cmdMin"), _cmdMin, double(0.0));
  privateNode->param(_name + std::string("cmdMax"), _cmdMax, double(0.0));

  privateNode->param(_name + std::string("wrapAngles"), _wrapAngles, bool(false));

  reset();
  _initialized = true;
  return true;
}

void PIDController::setGains(double p, double i, double d) {
  _pgain = p;
  _igain = i;
  _dgain = d;
}

void PIDController::updateNode(ros::NodeHandle *privateNode) {
  privateNode->setParam(_name + std::string("pgain"), _pgain);
  privateNode->setParam(_name + std::string("igain"), _igain);
  privateNode->setParam(_name + std::string("dgain"), _dgain);
}

double PIDController::process(double desired, double measured, double dt, bool hasRate, double measuredRate) {
  double command = 0.0;

  double error = desired - measured;
  if (_wrapAngles) {
    while (error > M_PI) {
      error -= 2 * M_PI;
    }
    while (error < -M_PI) {
      error += 2 * M_PI;
    }
  }

  double rate = measuredRate;
  if (!hasRate && _hasPreviousMeasurement) {
    rate = (measured - _prevMeasurement) / dt;
  }

  double desiredRate = 0.0;

  _pidMsg.error = error;
  _pidMsg.error_rate = (desiredRate - rate);
  _pidMsg.measured = measured;
  _pidMsg.desired = desired;
  _pidMsg.proportional = _pgain * error;
  _pidMsg.integral += _igain * error * dt;
  if (_pidMsg.integral > _imax) {
    _pidMsg.integral = _imax;
  }
  if (_pidMsg.integral < (-1.0 * _imax)) {
    _pidMsg.integral = -1.0 * _imax;
  }
  _pidMsg.derivative = _dgain * _pidMsg.error_rate;

  command = _pidMsg.proportional + _pidMsg.integral + _pidMsg.derivative;

  if (_cmdClip) {
    if (command < _cmdMin) {
      command = _cmdMin;
    }
    if (command > _cmdMax) {
      command = _cmdMax;
    }
  }

  _prevMeasurement = measured;
  _hasPreviousMeasurement = true;

  _pidMsg.command = command;
  _pidPub.publish(_pidMsg);

  return command;
}

void PIDController::reset() {
  _hasPreviousMeasurement = false;
  _pidMsg.integral = 0.0;
}

}
}
