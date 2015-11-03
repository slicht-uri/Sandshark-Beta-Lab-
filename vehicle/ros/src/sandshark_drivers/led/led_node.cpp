#include <ros/ros.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>
#include <sandshark_msgs/LEDStatus.h>
#include <sandshark_msgs/LEDCommand.h>
#include "SX1509.h"

#include <queue>

#define IR_PIN      0
#define STROBE_PIN   1
#define BLUE_PIN    3
#define GREEN_PIN   5
#define RED_PIN     6
#define AMBER_PIN   7

using namespace std;
namespace bluefin {
namespace sandshark {

class LEDControlDriver: public DriverBase {
  private:
    ros::Publisher _ledPub;
    sandshark_msgs::LEDStatus _ledMsg;

    uint8_t _ledStatus[8];

    bool _inErrorState;

    ros::Subscriber _blueSub;
    ros::Subscriber _redSub;
    ros::Subscriber _amberSub;
    ros::Subscriber _greenSub;
    ros::Subscriber _strobeSub;
    ros::Subscriber _irSub;

    ros::Rate * _rate;

    string _i2cPort;

    SX1509 *_ledcontrol;
    int _enableGPIOPin;

    // Queue of led actions to do in doRun(). Each pair is
    // a pair from the led number to the led command defined
    // in the led command message
    // the code must be structured this way because of
    // bug #57 (threading race conditions)
    std::queue<std::pair<int, int> > _ledActionQueue;

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    LEDControlDriver() :
        DriverBase("LEDDriver", "ledcontrol"), _inErrorState(false) {
    }

    uint8_t controlLED(const int pin, const int command);
    void msgBlueCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg);
    void msgRedCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg);
    void msgAmberCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg);
    void msgGreenCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg);
    void msgStrobeCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg);
    void msgIRCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg);

    void handleSleep();
};

uint8_t LEDControlDriver::controlLED(const int pin, const int command) {

  if (_ledcontrol == 0) {
    return 0;
  }

  bool result = true;

  uint8_t ret = 0;

  if (command == sandshark_msgs::LEDCommand::COMMAND_OFF) {
    result = result && _ledcontrol->blink(pin, 0, 0, 0, 0, 0, 0);
    result = result && _ledcontrol->pwm(pin, 0x00);
    ret = sandshark_msgs::LEDStatus::STATUS_OFF;
  } else if (command == sandshark_msgs::LEDCommand::COMMAND_ON) {
    result = result && _ledcontrol->blink(pin, 0, 0, 0, 0, 0, 0);
    result = result && _ledcontrol->pwm(pin, 0xFF);
    ret = sandshark_msgs::LEDStatus::STATUS_ON;
  } else if (command == sandshark_msgs::LEDCommand::COMMAND_BLINK_FAST) {
    result = result && _ledcontrol->blink(pin, 1, 1, 0, 255, 0, 0);
    ret = sandshark_msgs::LEDStatus::STATUS_BLINK_FAST;
  } else if (command == sandshark_msgs::LEDCommand::COMMAND_BLINK_SLOW) {
    result = result && _ledcontrol->blink(pin, 1, 9, 0, 255, 0, 0);
    ret = sandshark_msgs::LEDStatus::STATUS_BLINK_SLOW;
  } else {
    ret = sandshark_msgs::LEDStatus::STATUS_BAD_COMMAND;
    ROS_DEBUG("LEDControlDriver: Received unknown command for LED: Strobe.");
  }

  if (!result) {
    _inErrorState = true;
  }

  return ret;
}

void LEDControlDriver::msgIRCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg) {
  _ledActionQueue.push(std::pair<int, int>(IR_PIN, msg->command));
}

void LEDControlDriver::msgStrobeCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg) {
  _ledActionQueue.push(std::pair<int, int>(STROBE_PIN, msg->command));
}

void LEDControlDriver::msgBlueCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg) {
  _ledActionQueue.push(std::pair<int, int>(BLUE_PIN, msg->command));
}

void LEDControlDriver::msgGreenCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg) {
  _ledActionQueue.push(std::pair<int, int>(GREEN_PIN, msg->command));
}

void LEDControlDriver::msgRedCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg) {
  _ledActionQueue.push(std::pair<int, int>(RED_PIN, msg->command));
}

void LEDControlDriver::msgAmberCallback(const sandshark_msgs::LEDCommand::ConstPtr & msg) {
  _ledActionQueue.push(std::pair<int, int>(AMBER_PIN, msg->command));
}

void LEDControlDriver::startupInitCallback() {
  _blueSub = _publicNode->subscribe("/ledcontrol/blue", 1, &LEDControlDriver::msgBlueCallback, this);
  _redSub = _publicNode->subscribe("/ledcontrol/red", 1, &LEDControlDriver::msgRedCallback, this);
  _amberSub = _publicNode->subscribe("/ledcontrol/amber", 1, &LEDControlDriver::msgAmberCallback, this);
  _greenSub = _publicNode->subscribe("/ledcontrol/green", 1, &LEDControlDriver::msgGreenCallback, this);
  _strobeSub = _publicNode->subscribe("/ledcontrol/strobe", 1, &LEDControlDriver::msgStrobeCallback, this);
  _irSub = _publicNode->subscribe("/ledcontrol/ir", 1, &LEDControlDriver::msgIRCallback, this);

  _ledPub = _publicNode->advertise<sandshark_msgs::LEDStatus>("ledstatus", 10);

  _privateNode->param("i2cPort", _i2cPort, std::string("/dev/i2c-10"));
  _privateNode->param("enableGPIOPin", _enableGPIOPin, (int) 416);

  if (!gpioInit(_enableGPIOPin, false)) {
    ROS_ERROR("GPIO Init Failed! Continuing since it may already exist");
  }

  for(int i = 0; i < 8; i++) {
    _ledStatus[i] = 0;
  }
  _ledStatus[BLUE_PIN] = 1;

  _rate = new ros::Rate(10);
}

bool LEDControlDriver::doInitialize() {
  cleanup();

  //Enable the I2C buffers
  if (!gpioWrite(_enableGPIOPin, '1')) {
    setErrorMessage("Failed to enable LED I2C Buffers!");
    return false;
  }

  ROS_INFO("Using I2C Port %s", _i2cPort.c_str());
  _ledcontrol = new SX1509(_i2cPort.c_str());

  //initialize led control
  ROS_INFO("Calling ledcontrol init!");
  bool result = _ledcontrol->init();
  if (!result) {
    setErrorMessage("LED init failed!\n");
    return false;
  }

  result = true;
  //Turn off all LEDs for now
  result = result && _ledcontrol->pwm(0, 0x00);
  result = result && _ledcontrol->pwm(1, 0x00);
  result = result && _ledcontrol->pwm(2, 0x00);
  result = result && _ledcontrol->pwm(3, 0x00);
  result = result && _ledcontrol->pwm(4, 0x00);
  result = result && _ledcontrol->pwm(5, 0x00);
  result = result && _ledcontrol->pwm(6, 0x00);
  result = result && _ledcontrol->pwm(7, 0x00);

  ROS_INFO("LED init success");
  return result;
}

void LEDControlDriver::cleanup() {
  //Shutoff the buffers
  gpioWrite(_enableGPIOPin, '0');

  if (_ledcontrol) {
    delete _ledcontrol;
    _ledcontrol = 0;
  }
}

bool LEDControlDriver::doRun() {

  while (_ledActionQueue.size() != 0) {
    std::pair<int, int> action = _ledActionQueue.front();
    _ledActionQueue.pop();
    uint8_t ret = controlLED(action.first, action.second);
    if (!_inErrorState) {
      _ledStatus[action.first] = ret;
    }

  }
  _ledMsg.blue_status = _ledStatus[BLUE_PIN];
  _ledMsg.red_status = _ledStatus[RED_PIN];
  _ledMsg.amber_status = _ledStatus[AMBER_PIN];
  _ledMsg.green_status = _ledStatus[GREEN_PIN];
  _ledMsg.strobe_status = _ledStatus[STROBE_PIN];
  _ledMsg.ir_status = _ledStatus[IR_PIN];

  _ledPub.publish(_ledMsg);

  //return the inversion so _inErrorState==true means we are in an error state
  return !_inErrorState;
}

void LEDControlDriver::handleSleep() {
  _rate->sleep();
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::LEDControlDriver ld;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) ld, argc, argv);
}
