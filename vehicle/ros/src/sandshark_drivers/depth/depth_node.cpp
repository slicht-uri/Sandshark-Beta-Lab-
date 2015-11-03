#include "MS89BSD.h"

#include <ros/ros.h>
#include <sandshark_msgs/Depth.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class DepthDriver: public DriverBase {
  private:
    MS89BSD *_i2cInterface;

    ros::Publisher _depthPub;
    sandshark_msgs::Depth _depthMsg;

    string _i2cPort;
    int _enableGPIOPin;

    ros::Rate * _rate;

    float convertATMtoDBar(float & pressure_in_atm);

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    DepthDriver() :
        DriverBase("DepthDriver", "depth"), _i2cInterface( NULL) {
    }

    void handleSleep();
    virtual ~DepthDriver() {
    }
};

void DepthDriver::startupInitCallback() {
  _depthPub = _publicNode->advertise<sandshark_msgs::Depth>("depth", 1);

  _privateNode->param("i2cPort", _i2cPort, std::string("/dev/i2c-4"));
  _privateNode->param("enableGPIOPin", _enableGPIOPin, (int) 417);

  if (!gpioInit(_enableGPIOPin, false)) {
    ROS_ERROR("GPIO Init Failed! Continuing since it may already exist");
  }

  _rate = new ros::Rate(10);
}

void DepthDriver::cleanup() {
  //Shut off the buffers
  gpioWrite(_enableGPIOPin, '0');

  if (_i2cInterface) {
    delete _i2cInterface;
    _i2cInterface = 0;
  }
}

bool DepthDriver::doInitialize() {
  cleanup();

  //Enable the I2C buffers
  if (!gpioWrite(_enableGPIOPin, '1')) {
    ROS_ERROR("Failed to enable LED I2C Buffers!");
    return false;
  }

  _i2cInterface = new MS89BSD(_i2cPort.c_str(), 0x77);
  return _i2cInterface->init();
}

bool DepthDriver::doRun() {
  float pressure, temperature;

  if (!_i2cInterface->Read(&pressure, &temperature)) {
    return false;
  }

  _depthMsg.pressure = convertATMtoDBar(pressure);
  _depthMsg.temperature = temperature;
  _depthPub.publish(_depthMsg);

  return true;
}

void DepthDriver::handleSleep() {
  _rate->sleep();
}

float DepthDriver::convertATMtoDBar(float & pressure_in_atm) {
  return pressure_in_atm * (101325.0f / 10000.0f);
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::DepthDriver dd;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) dd, argc, argv);
}
