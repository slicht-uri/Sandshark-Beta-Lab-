#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <sstream>
#include <time.h>
#include <deque>
#include <math.h>
#include <serial.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include <sandshark_msgs/Altitude.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

//Period in ms for the main loop
#define ECH_LOOP_PERIOD_MS 1000

//Size constants for the switch command
#define ECH_SWITCH_CMD_LEN          27
#define ECH_SHOT_RET_MAX_LEN        512

#define ALTITUDE_FILTER_LEN         5

//Size constants for shot return
//Each includes a 3 byte header specifier, 9 bytes of variable header,
//either 0, 252, or 500 data bytes, and a 1 byte terminator.
#define ECH_LONG_SHOT_RET_LEN       513
#define ECH_MEDIUM_SHOT_RET_LEN     265 //we will use this length by default#define ECH_SHORT_SHOT_RET_LEN      13#define ECH_HEADER_LEN              12
#define ECH_HEADER_IDENTIFIER_LEN   3
#define ECH_TERMINATOR_LEN          1

//To indicate an invalid altitude
#define ECH_INVALID_ALTITUDE        -1.0

//defines for non-zero constant values in each switch command
#define ECH_SWITCH_HEADER_1         0xFE
#define ECH_SWITCH_HEADER_2         0x44
#define ECH_HEAD_ID                 0x11
#define ECH_ABSORPTION              0x14
#define ECH_FREQUENCY               0x00
#define ECH_SWITCH_TERMINATOR       0xFD

//defines indicating the locations of constant parameters in the switch message
#define ECH_SWITCH_HEADER_IDX       0
#define ECH_SWITCH_HEADID_IDX       2
#define ECH_SWITCH_ABSORPRION_IDX   10
#define ECH_SWITCH_FREQUENCY_IDX    25
#define ECH_SWITCH_TERMINATOR_IDX   26

//defines indicating the locations of variable parameters in the switch message
#define ECH_SWITCH_RANGE_IDX        3
#define ECH_SWITCH_GAIN_IDX         8
#define ECH_SWITCH_PULSELEN_IDX     14
#define ECH_SWITCH_PMINRANGE_IDX    15
#define ECH_SWITCH_TRIGGER_IDX      18
#define ECH_SWITCH_POINTS_IDX       19
#define ECH_SWITCH_PROFILE_IDX      22
#define ECH_SWITCH_DELAY_IDX        24

//defines for the defaults for each variable parameter in the switch message
#define ECH_DEFAULT_SWITCH_GAIN         20
#define ECH_DEFAULT_SWITCH_PULSELEN     120
#define ECH_DEFAULT_SWITCH_PMINRANGE    10
#define ECH_DEFAULT_SWITCH_TRIGGER      0   //no external trigger#define ECH_DEFAULT_SWITCH_POINTS       25  //252 data points in shot return#define ECH_DEFAULT_SWITCH_PROFILE      0#define ECH_DEFAULT_SWITCH_DELAY        50  //experimentally determined
using namespace std;
namespace bluefin {
namespace sandshark {

class AltimeterDriver: public DriverBase {

    /***********************Member Structs, Enums and Types*********************/

    //struct to hold all of the data from an altimeter shot return
    struct EchoSounderReturnData {
        float Altitude;     //The filtered altitude calculated from the shot return
        int Status;         //The status returned by the shot return ()
        int Range;          //The set range of the echosounder as returned by the shot return
        int ProfileRange;   //The latest profile range va
        int DataBytes;      //The number of data bytes returned in the shot return
    };

  private:

    /*********************ROS-specific member variables**************************/
    ros::Publisher _altitudePub;
    sandshark_msgs::Altitude _altitudeMsg;

    ros::Rate * _rate;

    /******************************Member Variables*******************************/

    //used to conveniently store echosunder return data
    struct EchoSounderReturnData _shotReturnData;

    //to hold the switch command and the shot return
    char _switchCmdBuff[ECH_SWITCH_CMD_LEN];
    char _shotRetBuff[ECH_SHOT_RET_MAX_LEN];

    //configurable filter parameters
    int _ech_range;
    double _data_threshold;
    double _noise_threshold;

    int _enableGPIOPin;
    /*********Serial Port Specific Parameters********/
    //the fd of the port we will communicate over
    int _port;
    //the baud rate we will communicate using
    int _BAUD_RATE;
    //string name of the serial port we will communicate over
    string _SERIAL_PORT;
    //for 485 settings
    fd_set _rfds;

    bool _requestProfile;
    //for setting of port properties
    struct termios _tty_attributes;
    //timeout for our select call
    struct timeval _read_period;

    std::deque<float> _unfiltered_altitudes;
    int _num_altitudes;

    /******************************Member Functions*******************************/

    /**
     * Sends the switch command stored in _switchCmdBuff to the device.  A
     *  devicetimeoutwarning will be issue if a different number of bytes
     *  than expected is returned from send()
     * @return void
     */
    void sendSwitchCommand();

    /**
     * Receives and parses an incoming shot return message from the device.
     *  This gets called when select returns in the main loop. It will read
     *  the data in one byte at a time until a header has been identified,
     *  then read the appropriate number of bytes until the termination footer.
     *  If any unexpected data is received, this function will return 0
     * @return int 1 on successful read,  0 if unsuccessful
     */
    int parseIncomingShotReturn();

    /**
     * Fills in the sandshark_msgs::Altitude message with the data received in the
     *  latest shot return and publishes it
     * @return void
     */
    void publishShotReturn();

    /**
     * Computes and returns an altitude estimate from a passed data portion
     *  of a shot return.  This function uses an altitude estimation algorithm
     *  written by Amy Underwood (aunderwood@bluefinrobotics.com) and verified
     *  by Mathieu(?) Kemp (mkemp@bluefinrobotics.com). This threshold detecting
     *  algorithm compares the signal contained within each intensityBin to the
     *  threshold.  The lowest numbered bin whose signal exceeds the threshold
     *  will be considered the bin detecting altitude.  For now, the threshold
     *  value is the experimentally determined ECH_DATA_THRESHOLD
     * @param a unsigned char a[] the data portion of a shot return
     * @param numDataPoints size_t the number of data point in a
     * @return float the altitude estimate calculated from passed data
     */
    float computeAltitude(char a[], size_t numDataPoints);

    void storeAltitude(float f);

    float getFilteredAltitude();

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
    /******************TaskBase functions**********************/
    void startupInitCallback();
    bool doInitialize();
    bool doRun();

  public:
    AltimeterDriver() :
        DriverBase("AltimeterDriver", "altitude") {
    }
    void handleSleep();
};

void AltimeterDriver::startupInitCallback() {
  _altitudePub = _publicNode->advertise<sandshark_msgs::Altitude>("altitude", 10);

  _rate = new ros::Rate(1000.0 / ECH_LOOP_PERIOD_MS);
  _privateNode->param("BAUD_RATE", _BAUD_RATE, (int) 115200);
  _privateNode->param("SERIAL_PORT", _SERIAL_PORT, std::string("/dev/ttyMAX2"));

  _privateNode->param("echRange", _ech_range, (int) 20);
  _privateNode->param("dataThreshold", _data_threshold, (double) 4.85);
  _privateNode->param("noiseThreshold", _noise_threshold, (double) 0.3);

  _privateNode->param("enableGPIOPin", _enableGPIOPin, (int) 422);

  _privateNode->param("requestProfile", _requestProfile, (bool) true);

  int read_ms = (ECH_LOOP_PERIOD_MS * 0.75); // experimentally determined
  _read_period.tv_sec = read_ms / 1000.0;
  _read_period.tv_usec = (read_ms % 1000) * 1000.0;

  ROS_INFO("read_ms = %d _read_period sec = %ld usec %ld", read_ms, _read_period.tv_sec, _read_period.tv_usec);

  for (int i = 0; i < ECH_SWITCH_CMD_LEN; i++) {
    _switchCmdBuff[i] = 0;
  }

  for (int i = 0; i < ECH_SHOT_RET_MAX_LEN; i++) {
    _shotRetBuff[i] = 0;
  }

  //Set the non-zero values of the switch command which will never change
  _switchCmdBuff[ECH_SWITCH_HEADER_IDX] = ECH_SWITCH_HEADER_1;
  _switchCmdBuff[ECH_SWITCH_HEADER_IDX + 1] = ECH_SWITCH_HEADER_2;
  _switchCmdBuff[ECH_SWITCH_HEADID_IDX] = ECH_HEAD_ID;
  _switchCmdBuff[ECH_SWITCH_ABSORPRION_IDX] = ECH_ABSORPTION;
  _switchCmdBuff[ECH_SWITCH_FREQUENCY_IDX] = ECH_FREQUENCY;
  _switchCmdBuff[ECH_SWITCH_TERMINATOR_IDX] = ECH_SWITCH_TERMINATOR;

  //Set the defaults for Variable Params
  _switchCmdBuff[ECH_SWITCH_RANGE_IDX] = _ech_range;
  _switchCmdBuff[ECH_SWITCH_GAIN_IDX] = ECH_DEFAULT_SWITCH_GAIN;
  _switchCmdBuff[ECH_SWITCH_PULSELEN_IDX] = ECH_DEFAULT_SWITCH_PULSELEN;
  _switchCmdBuff[ECH_SWITCH_PMINRANGE_IDX] = ECH_DEFAULT_SWITCH_PMINRANGE;
  _switchCmdBuff[ECH_SWITCH_TRIGGER_IDX] = ECH_DEFAULT_SWITCH_TRIGGER;
  _switchCmdBuff[ECH_SWITCH_POINTS_IDX] = ECH_DEFAULT_SWITCH_POINTS;
  _switchCmdBuff[ECH_SWITCH_PROFILE_IDX] = _requestProfile ? 1 : ECH_DEFAULT_SWITCH_PROFILE;
  _switchCmdBuff[ECH_SWITCH_DELAY_IDX] = ECH_DEFAULT_SWITCH_DELAY;

  if (!gpioInit(_enableGPIOPin, false)) {
    ROS_WARN("Failed to init GPIO %d, hope it was already exported....", _enableGPIOPin);
  }
  _num_altitudes = 0;
}

bool AltimeterDriver::doInitialize() {
  ROS_DEBUG("Entering doInitialize");
  cleanup();

  //Enable the altimeter
  if (!gpioWrite(_enableGPIOPin, '1')) {
    ROS_ERROR("Failed to enable altimeter!");
    return false;
  }

  //Give the device some time to boot
  sleep(2);

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

  //Nothing else to do as far as testing goes, if we cannot read successfully
  //from the device, we will fail later

  ROS_INFO("Serial Port init on %s maybe set to baud %d is complete.", _SERIAL_PORT.c_str(), _BAUD_RATE);
  return true;
}

void AltimeterDriver::cleanup() {
  ROS_DEBUG("Entering cleanup");

  //Shut off the altimeter
  gpioWrite(_enableGPIOPin, '0');

  //close the serial connection, don't need to tell the device anything
  close(_port);

  _num_altitudes = 0;
}

bool AltimeterDriver::doRun() {
  ROS_DEBUG("In doRun");

  bool isError = false;

  //local copy of the timeval so that we can use it with select
  struct timeval tval_switchdelay;
  tval_switchdelay.tv_usec = _read_period.tv_usec;
  tval_switchdelay.tv_sec = _read_period.tv_sec;

  //send the switch command to the device
  sendSwitchCommand();

  //clear the set and then add the rfds descriptor to it
  FD_ZERO(&_rfds);
  FD_SET(_port, &_rfds);

  ROS_INFO("Calling select port = %d, setsize %d", _port, (int) FD_SETSIZE);
  //use select to block until the echosounder replies.  We should hang here
  //for _switchCmdBuff[ECH_SWITCH_DELAY_IDX]ish ms, as that is when the device
  //should reply
  int retval = select(FD_SETSIZE, &_rfds, NULL, NULL, &tval_switchdelay);
  //The altimeter sends the command very slowly
  usleep(ECH_LOOP_PERIOD_MS * 0.20);
  //see what we get from the select
  if (retval == -1) {
    setErrorMessage("Select returned -1");
    return false;
  } else if (retval) {
    //read in the data one byte at a time
    if (parseIncomingShotReturn() != 1) {
      //if something went wrong reading in the altitude, timeoutwarn
      //and make sure the altitude published this doRun iteration will
      //be -1 (invalid)
      deviceTimeoutWarning("Malformed shot return received");
      _shotReturnData.Altitude = ECH_INVALID_ALTITUDE;
      //flush the port as we may not have read in all the data
      if (tcflush(_port, TCIOFLUSH) < 0) {
        //if we cannot flush it, something has gone horribly wrong
        setErrorMessage("Unable to tcflush");
        isError = true;
      }
    }
  } else {
    deviceTimeoutWarning("Altimeter shot return not received within expected time");
  }

  //publish the shot return no matter if we could receive a new altitude or not,
  //because if we didn't, ECH_INVALID_ALTITUDE will be published
  publishShotReturn();
  return !isError;
}

void AltimeterDriver::handleSleep() {
  //sleep for whatever leftover time there is
  _rate->sleep();
}

float AltimeterDriver::computeAltitude(char a[], size_t numDataPoints) {

  //get the number of bins that we are currently using
  float numBins = (float) numDataPoints;

  //calculate the bin size in m/bin
  float binSize = (float) _switchCmdBuff[ECH_SWITCH_RANGE_IDX] / numBins;

  //estimate the background noise
  int iMin = (int) (_noise_threshold / binSize);
  int i = 0;
  float iSum = 0;
  for (i = 0; i <= iMin; i++) {
    iSum += a[i];
  }
  float backgroundNoise = iSum / (iMin + 1);

  //estimate the altitude
  int j;
  for (j = 0; j < numBins; j++) {
    if (a[i] >= backgroundNoise * _data_threshold) {
      return i * binSize;
    }
  }

  return ECH_INVALID_ALTITUDE;  //result indicating that we could not estimate a reliable altitude
}

int AltimeterDriver::parseIncomingShotReturn() {
  ROS_DEBUG("In parseIncomingShotReturn");
  int numBytes = 0;
  //attempt to read in the header
  int numreceived = receive(_shotRetBuff, ECH_HEADER_IDENTIFIER_LEN);
  if (numreceived != ECH_HEADER_IDENTIFIER_LEN) {
    //somehow we were not able to read in the appropriate number of bytes
    ROS_WARN("parseIncomingShotReturn read less than %d bytes", ECH_HEADER_IDENTIFIER_LEN);
    return 0;
  }

  //now check to see if the header we read is a valid one and see how many more
  //bytes should be in the message
  if (_shotRetBuff[0] != 'I' || _shotRetBuff[2] != 'X') {
    ROS_WARN("received garbage header");
    return 0; //indicating no valid data read
  }
  if (_shotRetBuff[1] == 'M') {
    numBytes = ECH_MEDIUM_SHOT_RET_LEN - ECH_HEADER_IDENTIFIER_LEN;
  } else if (_shotRetBuff[1] == 'G') {
    numBytes = ECH_LONG_SHOT_RET_LEN - ECH_HEADER_IDENTIFIER_LEN;
  } else if (_shotRetBuff[1] == 'P') {
    numBytes = ECH_SHORT_SHOT_RET_LEN - ECH_HEADER_IDENTIFIER_LEN;
  } else { //garbage header so forget everything and return
    ROS_WARN("received garbage header");
    return 0; //indicating no valid data read
  }
  //now we know how many bytes to read in for the message body.
  //We have already read in ECH_HEADER_IDENTIFIER_LEN bytes, so now
  int pos = ECH_HEADER_IDENTIFIER_LEN;
  numreceived = 0;
  int count = 0;
  while (numreceived < numBytes) {
    ssize_t ret = receive(&_shotRetBuff[pos], numBytes - numreceived);
    if (ret < 0) {
      ROS_INFO("Received returned %ld, errno %d:%s count %d", ret, errno, strerror(errno), count);
      count++;
      if ((count * 10) > 1000) {
        ROS_INFO("Partial message recv'd, timeout on rest of message count = %d", count);
        return 0;
      }
    } else {
      numreceived += ret;
      pos = ECH_HEADER_IDENTIFIER_LEN + numreceived;
    }
    usleep(ECH_LOOP_PERIOD_MS * 0.05);
    ROS_INFO("Receiving - ret = %ld - numrecv %d pos %d", ret, numreceived, pos);
  }
  //double check that we received the correct number of bytes
  if (numreceived != numBytes) {
    ROS_WARN("Unexpected number of bytes: %d read after header identifier.  "
        "Expected %d", numreceived, numBytes);
    return 0;
  }
  //double check that the footer byte is correct
  if (_shotRetBuff[numBytes + 2] != 0xFC) {
    ROS_WARN("Unexpected terminating byte: 0x%02x instead of: 0xFC", _shotRetBuff[numBytes + 2]);
    return 0;
  }
  //At this point, we're pretty sure that the data we read in is valid
  //we do not care about byte 3, 5, and 6
  _shotReturnData.Status = (int) _shotRetBuff[4];
  _shotReturnData.Range = (int) _shotRetBuff[7];
  _shotReturnData.ProfileRange = (int) (((_shotRetBuff[9] & 0x7E) >> 1) << 8)
      | ((_shotRetBuff[9] & 0x01) << 7 | (_shotRetBuff[8] & 0x7F));
  _shotReturnData.DataBytes = (int) (((_shotRetBuff[11] & 0x7E) >> 1) << 8)
      | ((_shotRetBuff[11] & 0x01) << 7 | (_shotRetBuff[10] & 0x7F));
  if (_shotRetBuff[1] == 'P') {
    //we are in profile mode so just convert the profile range
    //to m and publish it as altitude, as there will be no other altitude?
    _shotReturnData.Altitude = (float) _shotReturnData.ProfileRange / 100.0;
  } else {
    //The message data begins at index ECH_HEADER_LEN and ends at byte
    //before the terminator.  The number of data bytes equals the overall
    //message length minus the length of the terminator and the length of
    //the header
    int numDataBytes = numBytes + ECH_HEADER_IDENTIFIER_LEN -
    ECH_HEADER_LEN - ECH_TERMINATOR_LEN;
    _shotReturnData.Altitude = computeAltitude(&_shotRetBuff[ECH_HEADER_LEN], numDataBytes);
  }
  return 1; //indicating success

}

void AltimeterDriver::sendSwitchCommand() {
  ROS_INFO("Sending Switch Command...\n");
  int bytesSent = send(_switchCmdBuff, ECH_SWITCH_CMD_LEN);
  //check to ensure that we have sent the correct number of bytes
  if (bytesSent != ECH_SWITCH_CMD_LEN) {
    deviceTimeoutWarning("Send returned an unexpected number of bytes");
    ROS_WARN("Unexpected number: %d of bytes sent in switch command.  Expected %d\n", bytesSent, ECH_SWITCH_CMD_LEN);
  }
}

void AltimeterDriver::publishShotReturn() {

  //filter the altitude before publishing
  storeAltitude(_shotReturnData.Altitude);
  _altitudeMsg.altitude = getFilteredAltitude();
  _altitudeMsg.status = _shotReturnData.Status;
  _altitudeMsg.range = _shotReturnData.Range;
  _altitudeMsg.profile_range = _shotReturnData.ProfileRange;
  _altitudeMsg.data_bytes = _shotReturnData.DataBytes;
  //publish it
  _altitudePub.publish(_altitudeMsg);
}

size_t AltimeterDriver::send(const char txBuff[], size_t numBytes) {
  return write(_port, txBuff, numBytes);
}

ssize_t AltimeterDriver::receive(char rxBuff[], size_t numBytes) {
  ssize_t numreceived = read(_port, rxBuff, numBytes);
  return numreceived;
}

void AltimeterDriver::storeAltitude(float f) {
  if (_unfiltered_altitudes.size() >= ALTITUDE_FILTER_LEN) {
    _unfiltered_altitudes.pop_front();
  }
  _unfiltered_altitudes.push_back(f);
}

float AltimeterDriver::getFilteredAltitude() {
  float altitudeSum = 0.0;
  std::deque<float>::iterator it = _unfiltered_altitudes.begin();
  while (it != _unfiltered_altitudes.end()) {
    altitudeSum += *it;
    it++;
  }
  return (altitudeSum / _unfiltered_altitudes.size());
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::AltimeterDriver ad;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) ad, argc, argv);
}
