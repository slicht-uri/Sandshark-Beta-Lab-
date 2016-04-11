#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>
#include <sandshark_msgs/TailconeState.h>
#include <sandshark_msgs/TailconePowerUsage.h>
#include <sandshark_msgs/TailconeRaw.h>
#include <sandshark_msgs/TailconeVersion.h>
#include <sandshark_msgs/ObjectiveControlStatus.h>
#include <ros/ros.h>

#include "conversation.h"

#include <string>
#include <map>
#include <utility>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <queue>
#include <serial.h>

//defines for channel specifiers
#define TC_THRUSTER             "T"
#define TC_RUDDER               "R"
#define TC_ELEVATOR             "E"
#define TC_BATTERY              "B"

//defines for response specifiers
#define TC_ERROR                "E"
#define TC_UNDEFINED            "U"
#define TC_SUCCESS              "1"

//defines for thruster motor channel states
#define TC_THR_IDLE             "I"
#define TC_THR_STOPPED          "S"
#define TC_THR_RUNNING          "R"
#define TC_THR_ERROR            "E"

//defines for actuator motor channel states
#define TC_ACT_IDLE             "I"
#define TC_ACT_STOPPED          "S"
#define TC_ACT_MOVING           "M"
#define TC_ACT_HOLDING          "H"
#define TC_ACT_ERROR            "E"

//possible return values for leak status
#define TC_LEAK_LOW             0
#define TC_LEAK_HIGH            1

//Get Command/Response Headers
#define TC_GETPOS_HEADER        "GP"
#define TC_GETVEL_HEADER        "GV"
#define TC_GETSTATE_HEADER      "GS"
#define TC_GETERRORSTATUS_HEADER "GE"
#define TC_GETDRIVEMODE_HEADER  "GDM"  //Not currently implemented
#define TC_GETENV_HEADER        "QE"   //Not currently implemented
#define TC_GETTEMP_HEADER       "GT"   //Not currently implemented
#define TC_GETPRESS_HEADER      "GPS"  //Not currently implemented
#define TC_GETLEAK_HEADER       "GLD"
#define TC_GETV_HEADER          "V"
#define TC_GETI_HEADER          "I"
#define TC_GETILIMIT_HEADER     "GI"

//Set Command/Response Headers
#define TC_SETTARGETS_HEADER    "STRE"
#define TC_SETPOS_HEADER        "UP"
#define TC_SETVEL_HEADER        "UV"

//Other Command/Response Headers
#define TC_ALLOFF_HEADER        "AO"
#define TC_HOME_HEADER          "HM"
#define TC_VER_HEADER           "VER"

#define TC_COMMAND_HEADER       "#"
#define TC_REPLY_HEADER         "$"

#define TC_COMMAND_TERMINATOR   "\r\n"
#define TC_REPLY_TERMINATOR     "\r\n"

//Masks for what to home
#define TC_HOME_THRUSTER        0x01
#define TC_HOME_RUDDER          0x02
#define TC_HOME_ELEVATOR        0x04

#define TC_UNDEFINED_FLOAT      NAN

//for conversion
#define TC_MILLI_DIVISOR        1000

#define TC_HOME_TIMEOUT_MS      5000
#define TC_CMD_TIMEOUT_MS       1000
#define TC_CMD_NUM_RETRIES      1

//Size constants for conversations - 64 is from EE Firmware Spec 3.4
#define TC_RX_BUFF_LEN          64
#define TC_TX_BUFF_LEN          64

//size constant for PID gain array
#define TC_PID_GAIN_ARRAY_LEN   9

//indexes for PID gain array
#define TC_PID_GAIN_INDEX_PT    0
#define TC_PID_GAIN_INDEX_IT    1
#define TC_PID_GAIN_INDEX_DT    2
#define TC_PID_GAIN_INDEX_PR    3
#define TC_PID_GAIN_INDEX_IR    4
#define TC_PID_GAIN_INDEX_DR    5
#define TC_PID_GAIN_INDEX_PE    6
#define TC_PID_GAIN_INDEX_IE    7
#define TC_PID_GAIN_INDEX_DE    8

//Period in ms for the main loop
#define TC_LOOP_PERIOD_MS       50

namespace bluefin {
namespace sandshark {

class TailconeDriver: public DriverBase {
  private:

    /**************************Member Enums and Types*************************/
    //enum for possible error codes read from the device channels
    enum ErrorCode {
      TC_EC_SHORT,
      TC_EC_RESERVED,
      TC_EC_HALLDIRECTION,
      TC_EC_HALLEFFECT,
      TC_EC_OVERVOLTAGE,
      TC_EC_SHORT2,
      TC_EC_OVERCURRENT,
      TC_EC_OVERTEMPERATURE
    };

    //enum for the severity of read warnings from device channels
    enum ErrorSeverity {
      TC_ES_NONE, TC_ES_WARN, TC_ES_MISSION, TC_ES_FATAL
    };

    struct TailconeState {
        std::string elevatorState;
        std::string rudderState;
        std::string thrusterState;
    };

    //map for error handling
    typedef std::map<ErrorCode, std::pair<std::string, ErrorSeverity> > errorActionMap;

    /*********************ROS-specific member variables***********************/
    //tailcone dynamics publication
    ros::Publisher _actualStatePub;
    ros::Publisher _versionPub;
    sandshark_msgs::TailconeState _actualStateMsg;
    sandshark_msgs::TailconeVersion _versionMsg;

    ros::Subscriber _commandedStateSub;
    sandshark_msgs::TailconeState _commandedStateMsg;

    std::string _commandTopic;

    ros::Rate * _rate;

    /******************************Member Variables*******************************/
    //array for holding PID gains for all motors
    double _PIDGains[TC_PID_GAIN_ARRAY_LEN];

    //variables to store desired and current physical tailcone parameters
    float _desiredElevator;
    float _desiredRudder;
    float _desired_rpm;
    float _currentElevator;
    float _currentRudder;
    int _currentRPM;

    bool _bluefinFrame;
    TailconeState _tailconeState;

    ros::Time _lastCmdTime;
    bool _lastCmdValid;
    ros::Time _lastAOTime;
    ros::Time _lastStatusTime;

    //used to keep track of what to home during doInitialize
    int _initHomeSet;

    //map to be loaded up with error codes, strings and actions
    errorActionMap _errorMap;

    //the address of the tailcone
    std::string _tcAddress;

    //the maximum/minimum angle from homed that the elevator and rudder are allowed to travel
    double _maxRudderAngle;
    double _maxElevatorAngle;

    //Buffers for input and output messages
    unsigned char _txBuff[TC_TX_BUFF_LEN];
    char _rxBuff[TC_RX_BUFF_LEN];

    //timeout for message responses
    struct timeval _message_timeout;

    /*********Serial Port Specific Parameters********/
    //the fd of the port we will communicate over
    int _port;
    //the baud rate we will communicate using
    int _BAUD_RATE;
    //string name of the serial port we will communicate over
    std::string _SERIAL_PORT;
    //for 485 settings
    fd_set _rfds;
    //for setting of port properties
    struct termios _tty_attributes;

    int _enableGPIOPin;
    int _resetMotorGPIOPin;

    int _roundRobinCounter;
    int _decimationCounter;

    bool _abortedState;

    /******************************Member Functions*******************************/
    /**
     * A combination of sendMessage and receiveMessage.  Converse will send the
     *  passed message to the device and then listen for a reply for the passed
     *  timeout.  For isCrucial=false: If no response is received, it will
     *  repeat the process n times, where n is TC_CMD_NUM_RETRIES before giving
     *  up and returning false.  For isCrucial=true: If no response is received,
     *  it will repeat the process indefinitely until the driver fails due to device
     *  timeout warnings
     * @param txrxHeader std::string the header for both the command and
     *  response (ex TC_GETPOS_HEADER)
     * @param txBody std::string the body data to send the device (ex "21.2 36.5")
     * @param timeout struct timeval timeout for the receiveMessage call within
     * @param isCrucial bool described above
     * @return bool true if the correct number of bytes were sent to the device
     *  and false otherwise
     */
    bool converse(std::string txrxHeader, std::string txBody, std::string & rxBody, struct timeval timeout,
        bool isCrucial = false);
    bool converse(std::string txrxHeader, std::string txBody, TCResponse& response, struct timeval timeout,
        bool isCrucial = false);

    /**
     * Sends a properly formatted message to the device with the passed command
     *  header and passed body. returns false and throws a deviceTimeoutWarning
     *  if an unexpected number of bytes were sent
     * @param header std::string the command header to send (ex TC_GETPOS_HEADER)
     * @param body std::string the body data to send the device (ex "21.2 36.5")
     * @return bool true if the correct number of bytes were sent to the device
     *  and false otherwise
     */
    bool sendMessage(std::string header, std::string body);

    /**
     * Attempts to receive a properly formatted message from _port with the passed header.
     *  If at any point during receipt of the message it is discovered that either it is
     *  an incorrect message or there is corrupted data, a deviceTimeoutWarning will be thrown
     *  and the function will return false.  _port will also be flushed.
     * @param header std::string the command header we expect to receive (ex TC_GETPOS_HEADER)
     * @param body std::string& the body data of the response will be stored in this string
     * @param timeout struct timeval how long to wait listening for a reply before giving up
     *  and returning false
     * @return bool returns false in any scenario in which the data in body should not
     *  be used including if it is TC_UNDEFINED
     */
    bool receiveMessage(std::string header, std::string &body, struct timeval timeout);

    /**
     * Utility function abstraction for write() which prints what is written()
     *  to stdout in ascii format.  Writes to _port
     * @param txBuff char* The array to write
     * @param numBytes size_t The number of bytes to write
     * @return size_t the number of bytes written to _port
     */
    size_t send(const char txBuff[], size_t numBytes);

    /**
     * Utility function abstraction for read() which prints what is read()
     *  to stdout in ascii format. Reads from _port
     * @param rxBuff char* The array to store read data in
     * @param numBytes size_t The number of bytes to read
     * @return size_t the number of bytes read from _port
     */
    ssize_t receive(char rxBuff[], size_t numBytes);

    void cleanup();
  protected:

    /**
     * Send the all off command to the device.  The response form the device
     *  is always the same, so we return nothing
     * @param channel the channel to home.  From channel #defines at top
     * @return void the function can take up to TC_CMD_TIMEOUT_MS to return
     */
    void allOffConvo();

    /**
     * Attempts to home the specified channel.  Homing can take up to
     *  TC_HOME_TIMEOUT_MS, so this function may not return for that long
     * @param channel the channel to home.  From channel #defines at top
     * @return bool returns true if homing was successful and false if not
     */
    bool homeChannelConvo(std::string channel);

    /**
     * Convenience method that will home the thruster, rudder, and elevator
     * @return    true if all homing was successful, false if not
     */
    bool homeAll();

    /**
     * Send and receive the STRE command to the device.  This command sets the
     *  desired rudder angle, elevator angle, and thruster rpm and returns the
     *  current respective values.
     * @return bool returns true if homing was successful and false if not
     */
    bool streConvo();

    bool verConvo();

    /**
     * Reads and returns the current state status of the specified channel
     *  from the device.
     * @param channel std::string the channel to query the error status of.
     *  From channel #defines at top
     * @return enum ErrorCode representing the returned error status
     */
    std::string getStateConvo(std::string channel);

    /**
     * Reads and returns the current error status of the specified channel
     *  from the device.
     * @param channel std::string the channel to query the error status of.
     *  From channel #defines at top
     * @return enum ErrorCode representing the returned error status
     */
    int getErrorStatusConvo(std::string channel);

    //now that the new firmware spec has come out and all errors are
    //cripplingly severe,e

    /**
     * Reads and returns the measured voltage of the specified channel from
     *  the device
     * @param channel std::string the channel to query the voltage of.
     *  From channel #defines at top
     * @return float representing the read voltage in volts, TC_UNDEFINED_FLOAT
     *  if returned invalid
     */
    float getVoltageConvo(std::string channel);

    /**
     * Reads and returns the measured current of the specified channel from
     *  the device
     * @param channel std::string the channel to query the current of.
     *  From #defines at top
     * @return float representing the read current in amps, TC_UNDEFINED_FLOAT
     *  if returned invalid
     */
    float getCurrentConvo(std::string channel);

    void fixSPIModeConvo();

    /**
     * Looks up a passed error code in the _errorMap and returns the
     *  corresponding error severity
     * @param ec int The error code to look up in _errorMap
     * @return int the corresponding error severity or -1 if the passed error
     *  code could not be found in _errorMap
     */
    int getErrorSeverity(int ec);

    /**
     * Looks up a passed error code in the _errorMap and returns the
     *  corresponding error string
     * @param ec int The error code to look up in _errorMap
     * @return std::string the corresponding error string or "Unknown Error Code"
     *  if the passed error code could not be found in _errorMap
     */
    std::string getErrorString(int ec);

    /**
     * Fills the _errorMap (std::map<ErrorCode, std::pair<std::string, ErrorSeverity> >)
     *  with ErrorCodes, error strings, and error Severities.  Error codes are the
     *  enumerated values returned by the device for a requested channel.  Error Strings
     *  are descriptive ascii strings that correspond to error codes.  Error severities
     *  are enumerated values corresponding to error codes that indicate what action
     *  the driver should take in response to receiving the error code.  Currently,
     *  the actions(in dealWithErrorStatus()) are fail the driver, log a warning, or
     *  do nothing.
     * @return void
     */
    void fillErrorMap();

    /**
     * Function called in doRun that determines whether doRun should return false,
     *  putting the driver in the failed state, or true, indicating that the
     *  driver is currently healthy.  Looks up the error severity of the passed
     *  error code and ladders down some levels to see what to do for that severity.
     *  If it returns false, so should doRun.  If it returns true, so should doRun
     */
    bool dealWithErrorStatus(std::string channel, unsigned char ec[]);

    /******************TaskBase functions**********************/
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:

    TailconeDriver() :
        DriverBase("TailconeDriver", "tailcone") {
    }

    /**
     * Callback for receipt of sandshark_msgs::TailconeState message from dynamic control.
     * Commands the device with the received desired values, reads the device's
     * current rpm and angle values and publishes them
     */
    void msgCmdCallback(const sandshark_msgs::TailconeState::ConstPtr & msg);

    /**
     * Callback for receipt of objective control status
     */
    void msgObjectiveControlStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg);

    void handleSleep();
};

void TailconeDriver::msgCmdCallback(const sandshark_msgs::TailconeState::ConstPtr & msg) {
  //make the local copies of desired match the ones we just received
  _desired_rpm = msg->thruster;
  _desiredRudder = msg->rudder;
  _desiredElevator = (_bluefinFrame ? 1 : -1 ) * msg->elevator;

  _lastCmdTime = ros::Time::now();
  _lastCmdValid = true;
}

void TailconeDriver::msgObjectiveControlStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg) {
  _abortedState = msg->is_aborted;
}

void TailconeDriver::startupInitCallback() {

  //for publishing the thruster rpm, rudder angle, and elevator angle
  _actualStatePub = _publicNode->advertise<sandshark_msgs::TailconeState>("currentpos", 10);
  _versionPub = _publicNode->advertise<sandshark_msgs::TailconeVersion>("version", 10, true);

  //callback for receipt of desired thruster rpm and actuator angles message from DC
  _privateNode->param("command_topic", _commandTopic, std::string("dynamiccontrol"));
  _commandedStateSub = _publicNode->subscribe(std::string("/") + _commandTopic + std::string("/tailcone_command"), 1,
      &TailconeDriver::msgCmdCallback, this);

  //read in serial port specifics
  _privateNode->param("BAUD_RATE", _BAUD_RATE, (int) 115200);
  _privateNode->param("SERIAL_PORT", _SERIAL_PORT, std::string("/dev/ttyMAX1"));

  _privateNode->param("enableGPIOPin", _enableGPIOPin, (int) 421);
  _privateNode->param("resetMotorGPIOPin", _resetMotorGPIOPin, (int) 409);

  //read in the tailcone address
  _privateNode->param("tcAddress", _tcAddress, std::string("00"));

  //read in safeties for rudder and elevator angles (bi-directional)
  _privateNode->param("maxRudderAngle", _maxRudderAngle, 30.0);
  _privateNode->param("maxElevatorAngle", _maxElevatorAngle, 30.0);

  //add a trailing space (see sendMessage and receiveMessage)
  _tcAddress = _tcAddress + " ";

  //read in PID gains for Thruster, Rudder, and Elevator
  _privateNode->param("PtGain", _PIDGains[TC_PID_GAIN_INDEX_PT], (double) 0.0);
  _privateNode->param("ItGain", _PIDGains[TC_PID_GAIN_INDEX_IT], (double) 0.0);
  _privateNode->param("DtGain", _PIDGains[TC_PID_GAIN_INDEX_DT], (double) 0.0);
  _privateNode->param("PrGain", _PIDGains[TC_PID_GAIN_INDEX_PR], (double) 0.0);
  _privateNode->param("IrGain", _PIDGains[TC_PID_GAIN_INDEX_IR], (double) 0.0);
  _privateNode->param("DrGain", _PIDGains[TC_PID_GAIN_INDEX_DR], (double) 0.0);
  _privateNode->param("PeGain", _PIDGains[TC_PID_GAIN_INDEX_PE], (double) 0.0);
  _privateNode->param("IeGain", _PIDGains[TC_PID_GAIN_INDEX_IE], (double) 0.0);
  _privateNode->param("DeGain", _PIDGains[TC_PID_GAIN_INDEX_DE], (double) 0.0);

  //Warn on any unset PID gains.  Not a real problem but nice to know
  for (int i = 0; i < TC_PID_GAIN_ARRAY_LEN; i++) {
    if (_PIDGains[i] == 0.0) {
      ROS_DEBUG("Unset PID Gain at index %d", i);
    }
  }

  //rate in Hz
  _rate = new ros::Rate((double) TC_MILLI_DIVISOR / (double) TC_LOOP_PERIOD_MS);

  //default timeout for message responses
  int tsecs = (double) TC_CMD_TIMEOUT_MS / (double) TC_MILLI_DIVISOR;
  _message_timeout.tv_sec = tsecs;
  _message_timeout.tv_usec = ((double) TC_CMD_TIMEOUT_MS - tsecs * (double) TC_MILLI_DIVISOR)
      * (double) TC_MILLI_DIVISOR;

  //populate the error action map
  fillErrorMap();

  if (!gpioInit(_enableGPIOPin, false)) {
    ROS_ERROR("Failed to init GPIO %d, hope it was already exported....", _enableGPIOPin);
  }

  if (!gpioInit(_resetMotorGPIOPin, false)) {
    ROS_ERROR("Failed to init motor reset GPIO %d, hope it was already exported....", _resetMotorGPIOPin);
  }

  _port = -1;

  _lastCmdValid = false;

  _tailconeState.elevatorState = "";
  _tailconeState.rudderState = "";
  _tailconeState.thrusterState = "";

  _roundRobinCounter = 0;
  _decimationCounter = 0;
}

bool TailconeDriver::doInitialize() {
  ROS_DEBUG("Entering doInitialize");
  cleanup();

  //delay before re-enable
  usleep(20000);

  //Enable the motors
  if (!gpioWrite(_enableGPIOPin, '1')) {
    setErrorMessage("Failed to enable motors");
    ROS_ERROR("Failed to enable motors!");
    return false;
  }

  //delay before re-enable
  usleep(20000);

  if (!gpioWrite(_resetMotorGPIOPin, '1')) {
    setErrorMessage("Failed to reset motors");
    ROS_ERROR("Failed to resetMotor!");
    return false;
  }

  //Give board time to come up,print banner, and home all channels
  sleep(5);

  //************************Open the Serial Port********************************//
  _port = open(_SERIAL_PORT.c_str(), O_RDWR);
  if (_port < 0) {
    //error opening the serial port
    setErrorMessage("Unable to open serial port");
    ROS_ERROR("Unable to open serial port");
    return false;
  }

  bzero(&_tty_attributes, sizeof(_tty_attributes));
  // 8 data bits
  _tty_attributes.c_cflag |= CS8 | CLOCAL | CREAD;
  // Set non-canonical mode (ICANON unset)
  _tty_attributes.c_lflag &= ~(ICANON);
  // Minimum number of characters for non-canonical read.
  _tty_attributes.c_cc[VMIN] = 0;
  // Timeout in deciseconds for non-canonical read.
  _tty_attributes.c_cc[VTIME] = 1;

  // Set the baud rate, same for input and output
  speed_t baud = B115200;
  switch (_BAUD_RATE) {
  case (9600):
    baud = B9600;
    break;
  case (57600):
    baud = B57600;
    break;
  case (115200):
    baud = B115200;
    break;
  default:
    ROS_WARN("Unsupported baud rate %d, using 115200", _BAUD_RATE);
    break;
  }

  cfsetospeed(&_tty_attributes, baud);
  cfsetispeed(&_tty_attributes, baud);

  int tcsetret = tcsetattr(_port, TCSANOW, &_tty_attributes);
  if (tcsetret < 0) {
    ROS_ERROR("tcsetattr error");
    setErrorMessage("tcsetattr error");
    return false;
  }

  //flush input and output for good measure
  if (tcflush(_port, TCIOFLUSH) < 0) {
    ROS_ERROR("Unable to tcFlush");
    setErrorMessage("Unable to tcflush");
    return false;
  }
  //******************Serial Port Should be open/ready ********************//
  //basically a BIT
  _bluefinFrame = false; //verConvo will set this
  return homeAll();
  //return true;
}

void TailconeDriver::cleanup() {
  ROS_DEBUG("Entering Cleanup");

  if (_port >= 0) {
    close(_port);
  }

  //Disable the motors
  if (!gpioWrite(_enableGPIOPin, '0')) {
    setErrorMessage("Failed to enable motors");
    ROS_ERROR("Failed to enable motors!");
  }

  //Disable the motor Reset (inverted logic)
  if (!gpioWrite(_resetMotorGPIOPin, '0')) {
    setErrorMessage("Failed to reset motors");
    ROS_ERROR("Failed to resetMotor!");
  }

  ROS_INFO("Exiting Cleanup\n");

}

bool TailconeDriver::homeAll() {
  if (!homeChannelConvo(TC_THRUSTER)) {
    setErrorMessage("Unable to home thruster");
    ROS_ERROR("Unable to home thruster");
    return false;
  }
  if (!homeChannelConvo(TC_ELEVATOR)) {
    setErrorMessage("Unable to home elevator");
    ROS_ERROR("Unable to home rudder");
    return false;
  }
  if (!homeChannelConvo(TC_RUDDER)) {
    setErrorMessage("Unable to home rudder");
    ROS_ERROR("Unable to home elevator");
    return false;
  }
  if( !verConvo() ) {
    setErrorMessage("Cannot get TC FW Version");
    ROS_ERROR("Cannot get TC FW Version");
    return false;
  }
  return true;
}

bool TailconeDriver::doRun() {

  ROS_DEBUG("Entering doRun");
  bool success = true;

  ros::Time now = ros::Time::now();
  if (!_lastCmdValid || !((now - _lastCmdTime) < ros::Duration(1.0))) {
    //if dynamic control has died, we want to turn off the tailcone
    _desiredElevator = 0.0;
    _desiredRudder = 0.0;
    _desired_rpm = 0.0;
  }

  //set desired rpm and angles
  streConvo();
  _actualStatePub.publish(_actualStateMsg);

  //query state and power info
  _decimationCounter++;
  if (_decimationCounter >= 10) {
    _decimationCounter = 0;

    _roundRobinCounter++;
    std::string stateString = "";
    std::string errorString = "";
    int errorCode = 0;
    unsigned char errors[8] = { 0 };
    if (_roundRobinCounter == 1) {
      //thruster
      stateString = getStateConvo(TC_THRUSTER);
      //_thrusterStatusMsg.state = stateString;
      //only bother checking error codes if in an error state
      if (stateString.find(TC_THR_ERROR) != std::string::npos) {
        errorCode = getErrorStatusConvo(TC_THRUSTER);
        //if the error code is 255, we need to send special command
        //and wait until next time to process errors.
        if (errorCode == 255) {
          fixSPIModeConvo();
        } else {
          for (unsigned int i = 0; i < _errorMap.size(); i++) {
            errors[i] = (errorCode >> i) & 1;
          }
          success = (success && dealWithErrorStatus(TC_THRUSTER, errors));
        }
      }
      //_thrusterStatusPub.publish( _thrusterStatusMsg );
    } else if (_roundRobinCounter == 2) {
      //rudder
      stateString = getStateConvo(TC_RUDDER);
      //_rudderStatusMsg.state = stateString;
      if (stateString.find(TC_ACT_ERROR) != std::string::npos) {
        errorCode = getErrorStatusConvo(TC_RUDDER);
        for (unsigned int i = 0; i < _errorMap.size(); i++) {
          errors[i] = (errorCode >> i) & 1;
        }
        success = (success && dealWithErrorStatus(TC_RUDDER, errors));
      }
      //_rudderStatusPub.publish( _rudderStatusMsg );
    } else if (_roundRobinCounter == 3) {
      //elevator
      stateString = getStateConvo(TC_ELEVATOR);
      //_elevatorStatusMsg.state = stateString;
      if (stateString.find(TC_ACT_ERROR) != std::string::npos) {
        errorCode = getErrorStatusConvo(TC_ELEVATOR);
        for (unsigned int i = 0; i < _errorMap.size(); i++) {
          errors[i] = (errorCode >> i) & 1;
        }
        success = (success && dealWithErrorStatus(TC_ELEVATOR, errors));
      }
      //_elevatorStatusPub.publish( _elevatorStatusMsg );
      _roundRobinCounter = 0;
    }
  }
  // if error is fatal, return false so the driver will fail and
  // health will restart the driver
  return success;
}

void TailconeDriver::handleSleep() {
  _rate->sleep();
}

void TailconeDriver::fillErrorMap() {
  _errorMap.insert(std::make_pair(TC_EC_SHORT, std::make_pair("Short circuit protection triggered", TC_ES_FATAL)));
  _errorMap.insert(std::make_pair(TC_EC_RESERVED, std::make_pair("Reserved.  Unimplemented", TC_ES_FATAL)));
  _errorMap.insert(std::make_pair(TC_EC_HALLDIRECTION, std::make_pair("Hall direction discrepancy", TC_ES_MISSION)));
  _errorMap.insert(std::make_pair(TC_EC_HALLEFFECT, std::make_pair("Hall signal  error", TC_ES_FATAL)));
  _errorMap.insert(std::make_pair(TC_EC_OVERVOLTAGE, std::make_pair("Overvoltage protection triggered", TC_ES_FATAL)));
  _errorMap.insert(std::make_pair(TC_EC_SHORT2, std::make_pair("Short circuit protection triggered2", TC_ES_FATAL)));
  _errorMap.insert(std::make_pair(TC_EC_OVERCURRENT, std::make_pair("Overcurrent protection triggered", TC_ES_FATAL)));
  _errorMap.insert(std::make_pair(TC_EC_OVERTEMPERATURE, std::make_pair("Overtemperature detected", TC_ES_FATAL)));
}

int TailconeDriver::getErrorSeverity(int ec) {
  errorActionMap::iterator it = _errorMap.find(ErrorCode(ec));
  if (it == _errorMap.end()) {
    return -1;
  }
  return it->second.second;
}

std::string TailconeDriver::getErrorString(int ec) {
  errorActionMap::iterator it = _errorMap.find(ErrorCode(ec));
  if (it == _errorMap.end()) {
    return std::string("Unknown error code");
  }
  return it->second.first;
}

bool TailconeDriver::dealWithErrorStatus(std::string channel, unsigned char ec[]) {
  bool goodState = true;

  for (unsigned int i = 0; i < _errorMap.size(); i++) {
    if (ec[i] != 0) {
      //Deal with the error codes
      if (getErrorSeverity(i) == TC_ES_NONE) {
        //do nothing, everything is fine
      } else if (getErrorSeverity(i) < TC_ES_FATAL) {
        //log the error
        ROS_WARN("Channel %s Error: %s", channel.c_str(), getErrorString(i).c_str());
      } else if (getErrorSeverity(i) == TC_ES_FATAL) {
        //make doRun return false, as we want to indicate extreme failure
        setErrorMessage(std::string("Fatal " + getErrorString(i) + " error on channel " + channel));
        ROS_ERROR("%s", std::string("Fatal " + getErrorString(i) + " error on channel " + channel).c_str());
        goodState = false;
      } else {
        //do nothing, invalid code warning will come from elsewhere
      }
      if (channel == TC_THRUSTER) {
        //_thrusterStatusMsg.error = i;
        //_thrusterStatusMsg.error_string = getErrorString(i);
      } else if (channel == TC_RUDDER) {
        //_rudderStatusMsg.error = i;
        //_rudderStatusMsg.error_string = getErrorString(i);
      } else if (channel == TC_ELEVATOR) {
        //_elevatorStatusMsg.error = i;
        //_elevatorStatusMsg.error_string = getErrorString(i);
      }
    }
  }

  return goodState;
}

void TailconeDriver::allOffConvo() {
  TCValueResponse<int> response("all off");
  //definitely crucial to get this command through
  converse(TC_ALLOFF_HEADER, std::string(""), response, _message_timeout/*, true*/);
}

bool TailconeDriver::homeChannelConvo(std::string channel) {
  //homing a channel on the device may take up to TC_HOME_TIMEOUT_MS
  //to complete and reply, so we use a different timeout here
  struct timeval home_timeout;
  int tsecs = (double) TC_HOME_TIMEOUT_MS / 1000.0;
  home_timeout.tv_sec = tsecs;
  home_timeout.tv_usec = ((double) TC_HOME_TIMEOUT_MS - tsecs * 1000.0) * 1000.0;

  TCValueResponse<int> response("home");
  //homing messages occur in the initialize, so we cannot count on the driverbase
  //timeouts check to fail on these
  return converse(TC_HOME_HEADER, channel, response, home_timeout/*, true*/);

}

bool TailconeDriver::verConvo() {
  //string to hold the reply
  std::string rxBody = "";
  if( !converse(TC_VER_HEADER, std::string(""), rxBody, _message_timeout ) ) {
    return false;
  }
  boost::algorithm::erase_all(rxBody, "\n");
  ROS_INFO("RXBODY VER %s", rxBody.c_str());
  std::vector<std::string> vals;

  boost::split(vals, rxBody, boost::is_any_of(" "));

  /*for( std::vector<std::string>::iterator iter = vals.begin(); iter != vals.end(); ++iter ) {
    ROS_INFO( "Split Vals = %s", (*iter).c_str() );
  }*/

  if( vals.size() >= 6 ) {
    _versionMsg.PartNumber = vals[1];
    _versionMsg.HardwareRevision = vals[5];
    _versionMsg.SoftwareRevision = vals[3];
    std::vector<std::string> srvals;
    boost::split(srvals, vals[3], boost::is_any_of("."));
    _versionMsg.bluefinTailFrame = false;
    for( std::vector<std::string>::iterator iter = srvals.begin(); iter != srvals.end(); ++iter ) {
        ROS_INFO( "Split Vals = %s", (*iter).c_str() );
    }
    if( !srvals.empty() && atoi( srvals[0].c_str() ) >= 1 ) {
        _versionMsg.bluefinTailFrame = true;
    } 
    _bluefinFrame = _versionMsg.bluefinTailFrame;
    _versionPub.publish( _versionMsg );
  } else { 
    return false;
  }
  return true;
}

bool TailconeDriver::streConvo() {
  //form the command string
  char txString[16];

  //Firmware fixed rudder negation, no more -1 needed
  //since this is the last step before physically sending desired angles to the tailcone firmware,
  //clip them for safety
  if (_desiredElevator > _maxElevatorAngle) {
    _desiredElevator = _maxElevatorAngle;
  } else if (_desiredElevator < -1 * _maxElevatorAngle) {
    _desiredElevator = -1 * _maxElevatorAngle;
  }

  if (_desiredRudder > _maxRudderAngle) {
    _desiredRudder = _maxRudderAngle;
  } else if (_desiredRudder < -1 * _maxRudderAngle) {
    _desiredRudder = -1 * _maxRudderAngle;
  }


  if (!_abortedState) {
    sprintf(txString, "%d %.1f %.1f", (int) _desired_rpm, _desiredRudder, _desiredElevator);
  } else {
    //if we are aborted, only do 0
    sprintf(txString, "%d %.1f %.1f", 0, 0.0, 0.0);
  }


  ROS_INFO("Sending stre. %s", txString);
  //string to hold the reply
  std::string rxBody = "";
  if (!converse(TC_SETTARGETS_HEADER, txString, rxBody, _message_timeout)) {
    std::string errorString = "STRE failed.  Returned: " + rxBody;
    setErrorMessage(errorString);
    return false;
  }

  //boost::algorithm::erase_all(rxBody, "\n");
  std::vector<std::string> vals;

  boost::split(vals, rxBody, boost::is_any_of(" "));

  try {
    //want to boost lexical cast but it seems like +0 is giving it problems
    int trpm = 0;
    float ea = 0.0;
    float ra = 0.0;
    std::stringstream t(vals[1]);
    std::stringstream r(vals[2]);
    std::stringstream e(vals[3]);
    t >> trpm;
    r >> ra;
    e >> ea;
    _actualStateMsg.thruster = trpm;
    _actualStateMsg.rudder = ra;
    _actualStateMsg.elevator = ea;
    _actualStateMsg.commanded_rudder = _desiredRudder;
    _actualStateMsg.commanded_elevator = _desiredElevator;
    _actualStateMsg.commanded_thruster = _desired_rpm;
  } catch (...) {
    //we don't really care here, we'll know when we query status, but
    //even so lets make a deviceTimeoutWarning
    deviceTimeoutWarning("STRE response parse error");
  }

  return true;
}

std::string TailconeDriver::getStateConvo(std::string channel) {
  std::string rxBody = "";
  TCValueResponse<std::string> response("get state");
  if (!converse(TC_GETSTATE_HEADER, channel, rxBody, _message_timeout)) {
    return TC_UNDEFINED;
  }
  boost::algorithm::erase_all(rxBody, " ");
  boost::algorithm::erase_all(rxBody, "\n");
  ROS_INFO("RXBODY %s", rxBody.c_str());
  return rxBody;
}

int TailconeDriver::getErrorStatusConvo(std::string channel) {
  TCValueResponse<int> response("get error");
  converse(TC_GETERRORSTATUS_HEADER, channel, response, _message_timeout);
  return response.value;
}

float TailconeDriver::getCurrentConvo(std::string channel) {
  TCValueResponse<float> response("get current");
  if (!converse(TC_GETI_HEADER, channel, response, _message_timeout)) {
    return TC_UNDEFINED_FLOAT;
  }
  return response.value / TC_MILLI_DIVISOR;
}

float TailconeDriver::getVoltageConvo(std::string channel) {
  TCValueResponse<float> response("get voltage");
  if (!converse(TC_GETV_HEADER, channel, response, _message_timeout)) {
    return TC_UNDEFINED_FLOAT;
  }

  return response.value / TC_MILLI_DIVISOR;
}

void TailconeDriver::fixSPIModeConvo() {
  std::string rec;
  ROS_WARN("FIXING SPI MODE FIRMWARE ISSUE");
  converse("FCM", "ON", rec, _message_timeout, false);
}

bool TailconeDriver::converse(std::string txrxHeader, std::string txBody, std::string& rxBody, struct timeval timeout,
    bool isCrucial) {
  bool rxSuccess = false;
  int numTries = 0;
  //send the message until we receive a meaningful reply. Timeouts will
  //automatically be added in the receivemessage function, so we do not need
  //to worry about failing the driver here if we never hear back about a message
  //only send and try to receive TC_NUM_RETRIES time for non-crucial messages
  //For crucial messages, try until we die
  do {
    sendMessage(txrxHeader, txBody);
    rxSuccess = receiveMessage(txrxHeader, rxBody, timeout);
    numTries++;
  } while ((isCrucial && !rxSuccess) || (numTries < TC_CMD_NUM_RETRIES));

  return rxSuccess;
}

bool TailconeDriver::converse(std::string txrxHeader, std::string txBody, TCResponse& response, struct timeval timeout,
    bool isCrucial) {
  bool rxSuccess = false;
  int numTries = 0;
  std::string rxBody;

  //send the message until we receive a meaningful reply. Timeouts will
  //automatically be added in the receivemessage function, so we do not need
  //to worry about failing the driver here if we never hear back about a message
  //only send and try to receive TC_NUM_RETRIES time for non-crucial messages
  //For crucial messages, try until we die
  do {
    sendMessage(txrxHeader, txBody);
    rxSuccess = receiveMessage(txrxHeader, rxBody, timeout);
    numTries++;
  } while ((isCrucial && !rxSuccess) || (numTries < TC_CMD_NUM_RETRIES));

  if (rxSuccess) {
    response.parseResponse(rxBody);
    if (response.invalid) {
      deviceTimeoutWarning(response.error);
      return false;
    }
    return true;
  }

  return rxSuccess;
}

bool TailconeDriver::sendMessage(std::string header, std::string body) {
  std::string sendString = std::string(TC_COMMAND_HEADER) + _tcAddress + header + " " + body
      + std::string(TC_COMMAND_TERMINATOR);
  size_t bytesSent = send(sendString.c_str(), sendString.length());

  ROS_INFO("Sending string bytes sent = %d - %s", (int) bytesSent, sendString.c_str());
  if (bytesSent != sendString.length()) {
    ROS_WARN("%s Message: Unexpected number: %d of bytes sent.  Expected %d\n", header.c_str(), (int) bytesSent,
        (int) sendString.length());
    deviceTimeoutWarning("Unexpected number of bytes sent");
    return false;
  }
  return true;
}

bool TailconeDriver::receiveMessage(std::string header, std::string &body, struct timeval timeout) {
  //local copy of the timeval so that we can use it with select
  struct timeval tval_recvdelay;
  tval_recvdelay.tv_usec = timeout.tv_usec;
  tval_recvdelay.tv_sec = timeout.tv_sec;

  ssize_t numBytes;

  //clear the set and then add the rfds descriptor to it
  FD_ZERO(&_rfds);
  FD_SET(_port, &_rfds);

  ROS_INFO("Receiving time out = %d : %d", (int) tval_recvdelay.tv_sec, (int) tval_recvdelay.tv_usec);

  //use select to block until we receive a response
  int retval = select(FD_SETSIZE, &_rfds, NULL, NULL, &tval_recvdelay);

  //see what we get from the select
  if (retval == -1) {
    setErrorMessage("Select returned -1");
    return false;
  } else if (retval) {
    std::string wholeHeaderString = std::string(TC_REPLY_HEADER) + _tcAddress + header;
    size_t i;

    //read in the whole header
    for (i = 0; i < wholeHeaderString.length(); i++) {
      numBytes = receive(&_rxBuff[i], 1);
    }
    std::string recvHeader(_rxBuff, i);
    if (recvHeader != wholeHeaderString) {
      char outBuf[100];
      sprintf(outBuf, "Bad header from device. Expected '%s', got '%s'", wholeHeaderString.c_str(), recvHeader.c_str());
      deviceTimeoutWarning(outBuf);
      if (tcflush(_port, TCIOFLUSH) < 0) {
        ROS_WARN("Unable to tcflush after header");
      }
      return false;
    }

    //Now the header has been read in.  Time for the body
    //Read in characters until we see the first character of the terminator or until
    //we have read in x characters.

    std::string termString = std::string(TC_REPLY_TERMINATOR);
    char t1 = *termString.begin();
    size_t termLen = termString.length();

    i = wholeHeaderString.length();
    //read one byte at a time so we don't wait 200ms trying to read
    //all TC_RX_BUFF_LEN bytes every time.
    while (i < TC_RX_BUFF_LEN) {
      numBytes = receive(&_rxBuff[i], 1);

      if (numBytes != 1) {
        char outBuf[TC_RX_BUFF_LEN + 100];
        char temp[TC_RX_BUFF_LEN];
        std::string recvString(_rxBuff, i);
        //there was a seg fault if this was put in a std::string
        //so doing a memcpy here even though it feels awful
        memcpy(temp, _rxBuff, i * sizeof(char));
        temp[i + 1] = '\0';
        sprintf(outBuf, "Stopped receiving data from device before "
            "line terminator reached. Got %s\n", temp);
        deviceTimeoutWarning(outBuf);
        if (tcflush(_port, TCIOFLUSH) < 0) {
          ROS_WARN("Unable to tcflush after header");
        }
        return false;
      }

      //check for terminator
      if (_rxBuff[i] == t1) {
        bool terminate = true;
        size_t j;
        for (j = 1; j < termLen && i + j < TC_RX_BUFF_LEN; j++) {
          numBytes = receive(&_rxBuff[i + j], 1);

          if (numBytes != 1) {
            char outBuf[100];
            std::string recvString(_rxBuff, i + j);
            sprintf(outBuf, "Stopped receiving data from device before "
                "line terminator reached. Got %s\n", recvString.c_str());
            deviceTimeoutWarning(outBuf);
            if (tcflush(_port, TCIOFLUSH) < 0) {
              ROS_WARN("Unable to tcflush after header");
            }
            return false;
          }

          if (_rxBuff[i + j] != termString[j]) {
            terminate = false;
            break;
          }
        }
        i += j;
        if (terminate)
          break;
      }

      i++;
    }

    //check that we did not read too many characters
    if (i + 1 == TC_RX_BUFF_LEN) {
      deviceTimeoutWarning("Overrun of characters received");
      if (tcflush(_port, TCIOFLUSH) < 0) {
        ROS_WARN("Unable to tcflush in receivemessage2");
      }
      return false;
    }

    //put the received body into the pbr body string
    //from rxBuff[wholeHeaderString.length()] to rxBuff[i-1]
    for (unsigned int p = wholeHeaderString.length(); p < i; p++) {
      body.push_back(_rxBuff[p]);
    }

    //Finally check to see if the body is TC_UNDEFINED and return false if so
    if (body == TC_UNDEFINED) {
      deviceTimeoutWarning("Unknown message header received");
      return false; //return false because no useful data
    }
  } else {
    deviceTimeoutWarning("reply not received within select timeout");
    return false;
  }
  boost::algorithm::erase_all(body, "\n");
  return true;
}

size_t TailconeDriver::send(const char txBuff[], size_t numBytes) {
  return ::write(_port, &txBuff[0], numBytes);
}

ssize_t TailconeDriver::receive(char rxBuff[], size_t numBytes) {
  ssize_t numreceived = ::read(_port, rxBuff, numBytes);
  return numreceived;
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::TailconeDriver tc;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) tc, argc, argv);
}
