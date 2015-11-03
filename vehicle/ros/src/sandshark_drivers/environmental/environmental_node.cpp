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
#include <fcntl.h>

#include <ros/ros.h>
#include <sandshark_msgs/Environmental.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

#define ENV_PRESSURE_DEVFS_PATH "/sys/devices/virtual/input/input1/get_pressure_data"
#define ENV_TEMP_DEVFS_PATH "/sys/devices/virtual/input/input1/get_temperature_data"
#define ENV_TEMP_PRESS_BYTES 16

using namespace std;
namespace bluefin {
namespace sandshark {

class EnvironmentalDriver: public DriverBase {
  private:

    ros::Publisher _envPub;
    sandshark_msgs::Environmental _envMsg;

    ros::Rate * _rate;

    int _pressure_fd;
    int _temperature_fd;

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    EnvironmentalDriver() :
        DriverBase("EnvironmentalDriver", "environmental") {
    }

    void handleSleep();
    void handleShutdown() {
      cleanup();
    }
};

void EnvironmentalDriver::startupInitCallback() {

  ROS_DEBUG("Entering startupInitCallback");

  _envPub = _publicNode->advertise<sandshark_msgs::Environmental>("environmental", 1);

  _rate = new ros::Rate(10);

  //set files to invalid defaults
  _pressure_fd = -1;
  _temperature_fd = -1;
}

bool EnvironmentalDriver::doInitialize() {

  ROS_DEBUG("Entering doInitialize");

  //do a cleanup
  cleanup();

  //Verify that we are able to open fds for both pressure and temperature
  _pressure_fd = open(ENV_PRESSURE_DEVFS_PATH, O_RDONLY);
  if (_pressure_fd < 0) {
    //by returning false, _state will become TB_ERROR, so set error string
    setErrorMessage("Unable to open pressure file in doInitialize");
    return false;
  } else {
    ROS_DEBUG("Successfully opened pressure file");
    close(_pressure_fd);
  }

  _temperature_fd = open(ENV_TEMP_DEVFS_PATH, O_RDONLY);
  if (_temperature_fd < 0) {
    //by returning false, _state will become TB_ERROR, so set error string
    setErrorMessage("Unable to open temperature file in doInitialize");
    return false;
  } else {
    ROS_DEBUG("Successfully opened temperature file");
    close(_temperature_fd);
  }

  return true;
}

void EnvironmentalDriver::cleanup() {
  ROS_DEBUG("Cleaning Up");
  //make sure that the pressure and temperature files were closed
  close(_pressure_fd);
  close(_temperature_fd);
}

bool EnvironmentalDriver::doRun() {
  int ret = -1;
  char buf[ENV_TEMP_PRESS_BYTES];

  bool updateValues = false;

  //we must open the file, read it, then close or else it will not be written
  //to by whatever process writes the pressure to it.
  _pressure_fd = open(ENV_PRESSURE_DEVFS_PATH, O_RDONLY);
  ret = read(_pressure_fd, buf, ENV_TEMP_PRESS_BYTES);
  close(_pressure_fd);

  if (ret < 0) {
    deviceTimeoutWarning("Failed to read pressure");
  } else {
    updateValues = true;
    _envMsg.pressure = (double) ((float) atoi(buf) / 1000);
  }

  //we must open the file, read it, then close or else it will not be written
  //to by whatever process writes the temp to it.
  _temperature_fd = open(ENV_TEMP_DEVFS_PATH, O_RDONLY);
  ret = read(_temperature_fd, buf, ENV_TEMP_PRESS_BYTES);
  close(_temperature_fd);
  //verify that we were able to read a temperature from the file
  if (ret < 0) {
    deviceTimeoutWarning("Failed to read temperature");
  } else {
    updateValues = true;
    _envMsg.temperature = (double) ((float) atoi(buf) / 10);
  }

  if (updateValues) {
    _envPub.publish(_envMsg);
  }

  return true;
}

void EnvironmentalDriver::handleSleep() {
  _rate->sleep();
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::EnvironmentalDriver ed;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) ed, argc, argv);
}
