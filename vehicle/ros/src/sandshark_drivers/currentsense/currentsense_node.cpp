#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <string>
#include <strings.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <csdio.h>

#include <ros/ros.h>
#include <sandshark_msgs/LeakDetect.h>
#include <sandshark_msgs/CurrentSense.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

static const int ADC_base = 0x60;
static const int DAC_base = 0x40;

using namespace std;
namespace bluefin {
namespace sandshark {

class CurrentSenseDriver: public DriverBase {
  private:
    int _csdioFd;
    volatile struct csdio_cmd52_ctrl_t _ctrlSDIO;

    ros::Publisher _currentsensePub;
    sandshark_msgs::CurrentSense _currentsenseMsg;

    ros::Publisher _leakPub;
    sandshark_msgs::LeakDetect _leakMsg;
    double _leakThreshold;

    bool readValue(uint8_t channel, double & value);
    double convertToVoltage(double adcReading);
    double convertToCurrent(double voltage, double C, double K, double R);

    ros::Rate * _rate;

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    CurrentSenseDriver() :
        DriverBase("CurrentSenseDriver", "currentsense") {
    }

    void handleSleep();
    void handleShutdown() {
      cleanup();
    }
};

void CurrentSenseDriver::startupInitCallback() {
  _currentsensePub = _publicNode->advertise<sandshark_msgs::CurrentSense>("current_sense", 1);
  _leakPub = _publicNode->advertise<sandshark_msgs::LeakDetect>("leak_detect", 1);

  _privateNode->param("leakThreshold", _leakThreshold, (double) 46000);

  _rate = new ros::Rate(10);
}

bool CurrentSenseDriver::doInitialize() {
  cleanup();

  _csdioFd = open("/dev/csdiof1", O_RDWR);
  if (_csdioFd < 0) {
    ROS_WARN("Unable to open current sense device");
    return false;
  }
  // ADC: reset all
  _ctrlSDIO.m_write = 1;
  _ctrlSDIO.m_address = ADC_base + 0x01;
  _ctrlSDIO.m_data = 0x20;
  if (ioctl(_csdioFd, CSDIO_IOC_CMD52, &_ctrlSDIO) == -1) {
    return false;
  }
  // ADC: channel config used for all inputs
  _ctrlSDIO.m_write = 1;
  _ctrlSDIO.m_address = ADC_base + 0x00;
  _ctrlSDIO.m_data = 0x07;  //current chan from previous command, 0x07 setups input a unipolar ref'd to GND
  //  test1.m_data = 0x06; //0x06 setups input a unipolar ref'd to COM
  if (ioctl(_csdioFd, CSDIO_IOC_CMD52, &_ctrlSDIO) == -1) {
    return false;
  }

  return true;
}

void CurrentSenseDriver::cleanup() {
  if (_csdioFd > 0) {
    close(_csdioFd);
  }
}

bool CurrentSenseDriver::readValue(uint8_t channel, double & value) {
  // ADC: read channel
  _ctrlSDIO.m_write = 1;
  _ctrlSDIO.m_address = ADC_base + 0x01;
  _ctrlSDIO.m_data = 0x40 + channel;
  if (ioctl(_csdioFd, CSDIO_IOC_CMD52, &_ctrlSDIO) == -1) {
    return false;
  }
  usleep(100000);

  // ADC: wait for completion
  _ctrlSDIO.m_write = 0;
  _ctrlSDIO.m_address = ADC_base + 0x02;
  if (ioctl(_csdioFd, CSDIO_IOC_CMD52, &_ctrlSDIO) == -1) {
    return false;
  }

  while ((_ctrlSDIO.m_data & 0x02) != 0) {
    ROS_INFO("ADC waiting data=%x", _ctrlSDIO.m_data);
    if (ioctl(_csdioFd, CSDIO_IOC_CMD52, &_ctrlSDIO) == -1) {
      return false;
    }
  }
  ROS_INFO("ADC want channel %x got channel=%x  ", channel, (_ctrlSDIO.m_data & 0x1C) >> 2);

  // ADC: read value
  _ctrlSDIO.m_write = 0;
  _ctrlSDIO.m_address = ADC_base + 0x04;
  if (ioctl(_csdioFd, CSDIO_IOC_CMD52, &_ctrlSDIO) == -1) {
    return false;
  }
  int reading = _ctrlSDIO.m_data & 0xff;
  _ctrlSDIO.m_write = 0;
  _ctrlSDIO.m_address = ADC_base + 0x05;
  if (ioctl(_csdioFd, CSDIO_IOC_CMD52, &_ctrlSDIO) == -1) {
    return false;
  }
  reading = (_ctrlSDIO.m_data << 8) | reading;
  ROS_INFO(" %x\n", reading);

  value = reading * 1.0;
  return true;
}

bool CurrentSenseDriver::doRun() {

  double m2mR = 0.05;
  double payloadR = 0.047;
  double motionR = 0.015;
  double altimeterR = 0.33;

  double m2mK = 50.0;
  double payloadK = 20.0;
  double motionK = 50.0;
  double altimeterK = 20.0;

  double m2mC = 1.2;
  double payloadC = 1.3;
  double motionC = 1.2;
  double altimeterC = 1.3;

  double motor_current;
  double payload_current;
  double altimiter_current;
  double motion_current;
  double m2m_current;
  double leakDetect;

  if (!readValue(0, motor_current)) {
    setErrorMessage("Failed to read motor current");
    return false;
  }

  if (!readValue(1, payload_current)) {
    setErrorMessage("Failed to read payload current");
    return false;
  }

  if (!readValue(2, altimiter_current)) {
    setErrorMessage("Failed to read altimeter current");
    return false;
  }

  if (!readValue(3, motion_current)) {
    setErrorMessage("Failed to read motion current");
    return false;
  }

  if (!readValue(4, m2m_current)) {
    setErrorMessage("Failed to read m2m current");
    return false;
  }

  if (!readValue(5, leakDetect)) {
    setErrorMessage("Failed to read Leak Detect");
    return false;
  }

  //Still need to convert from voltage reading to a current value
  _currentsenseMsg.motor_current = convertToVoltage(motor_current);

  _currentsenseMsg.payload_current = convertToCurrent(convertToVoltage(payload_current), payloadC, payloadK, payloadR);
  _currentsenseMsg.motion_current = convertToCurrent(convertToVoltage(motion_current), motionC, motionK, motionR);
  _currentsenseMsg.altimiter_current = convertToCurrent(convertToVoltage(altimiter_current), altimeterC, altimeterK,
      altimeterR);
  _currentsenseMsg.m2m_current = convertToCurrent(convertToVoltage(m2m_current), m2mC, m2mK, m2mR);
  _currentsenseMsg.altimiter_current = convertToVoltage(altimiter_current);
  _currentsenseMsg.motion_current = convertToVoltage(motion_current);
  _currentsenseMsg.m2m_current = convertToVoltage(m2m_current);

  _currentsensePub.publish(_currentsenseMsg);

  //Should we convert this to voltage first? Not sure if it is worth doing, the voltage reading may be worthless
  _leakMsg.leak = (leakDetect < _leakThreshold);
  _leakMsg.voltage = leakDetect;
  _leakPub.publish(_leakMsg);

  return true;
}

void CurrentSenseDriver::handleSleep() {
  _rate->sleep();
}

double CurrentSenseDriver::convertToVoltage(double adcReading) {
  return (adcReading / (pow(2.0, 12.0))) * 2.5; //12 bit ADC with 2.5 V reference
}

double CurrentSenseDriver::convertToCurrent(double voltage, double C, double K, double R) {
  return ((voltage / C) / K) / R;
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::CurrentSenseDriver cd;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) cd, argc, argv);
}
