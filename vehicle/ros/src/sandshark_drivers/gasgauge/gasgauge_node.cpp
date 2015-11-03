#include "BQ34Z100.h"

#include <ros/ros.h>
#include <sandshark_msgs/GasGauge.h>
#include <sandshark_msgs/LEDCommand.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class GasGaugeDriver: public DriverBase {
  private:
    BQ34Z100 *_i2cInterface;

    ros::Publisher _gaugePub;
    sandshark_msgs::GasGauge _gaugeMsg;

    ros::Publisher _blueLEDPub;
    sandshark_msgs::LEDCommand _blueLEDMsg;

    string _i2cPort;
    int _enableGPIOPin;
    double _lastCurrent;

    ros::Rate * _rate;

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    GasGaugeDriver() :
        DriverBase("GasGaugeDriver", "gasgauge"), _i2cInterface( NULL) {
    }

    void handleSleep();
    virtual ~GasGaugeDriver() {
    }
};

void GasGaugeDriver::startupInitCallback() {
  _gaugePub = _publicNode->advertise<sandshark_msgs::GasGauge>("gasgauge", 10);
  _blueLEDPub = _publicNode->advertise<sandshark_msgs::LEDCommand>("/ledcontrol/blue", 1);

  _privateNode->param("i2cPort", _i2cPort, std::string("/dev/i2c-2"));
  _privateNode->param("enableGPIOPin", _enableGPIOPin, (int) 418);

  if (!gpioInit(_enableGPIOPin, false)) {
    ROS_ERROR("GPIO Init Failed! Continuing since it may already exist");
  }
  _rate = new ros::Rate(10);
}

void GasGaugeDriver::cleanup() {
  //Shut off the buffers
  gpioWrite(_enableGPIOPin, '0');

  if (_i2cInterface) {
    delete _i2cInterface;
    _i2cInterface = 0;
  }
}

bool GasGaugeDriver::doInitialize() {
  cleanup();

  //Enable the I2C buffers
  if (!gpioWrite(_enableGPIOPin, '1')) {
    ROS_ERROR("Failed to enable LED I2C Buffers!");
    return false;
  }

  _i2cInterface = new BQ34Z100();
  return _i2cInterface->initialize(_i2cPort.c_str());

  //on start assume discharging
  _lastCurrent = -1.0;
}

bool GasGaugeDriver::doRun() {
  float state_of_charge, remaining_capacity, full_charge_capacity, voltage, current, temperature;

  bool success = _i2cInterface->Read(&state_of_charge, &remaining_capacity, &full_charge_capacity, &voltage, &current,
      &temperature);

  if (success) {
    _gaugeMsg.state_of_charge = state_of_charge;
    _gaugeMsg.remaining_capacity = remaining_capacity;
    _gaugeMsg.full_charge_capacity = full_charge_capacity;
    _gaugeMsg.percent_remaining = (int) floor(remaining_capacity / full_charge_capacity * 100.0);
    _gaugeMsg.voltage = voltage;
    _gaugeMsg.average_current = current;
    if(current > 0 && _lastCurrent < 0) {
      //switched to charging, set blue LED to pulse slow
      _blueLEDMsg.command = sandshark_msgs::LEDCommand::COMMAND_BLINK_SLOW;
      _blueLEDPub.publish(_blueLEDMsg);
    } else if(current < 0 && _lastCurrent > 0) {
      //switched to discharging, LED to off (on)
      _blueLEDMsg.command = sandshark_msgs::LEDCommand::COMMAND_OFF;
      _blueLEDPub.publish(_blueLEDMsg);
    }
    _lastCurrent = current;
    _gaugeMsg.temperature = temperature;
    _gaugePub.publish(_gaugeMsg);
  } else {
    deviceTimeoutWarning("GasGaugeDriver did not successfully read");
  }

  //always return true because DeviceBase will take care of device timeouts
  return true;
}

void GasGaugeDriver::handleSleep() {
  _rate->sleep();
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::GasGaugeDriver gg;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) gg, argc, argv);
}
