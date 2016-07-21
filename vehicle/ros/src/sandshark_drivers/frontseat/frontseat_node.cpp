#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <sstream>
#include <deque>
#include <time.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "boost/algorithm/string/predicate.hpp"

#include "bfMessage.h"
#include "bpMessage.h"

#include "ros/ros.h"
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>
#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/Velocities.h>
#include <sandshark_msgs/Speed.h>
#include <sandshark_msgs/DynamicControlCommand.h>
#include <sandshark_msgs/PayloadStatus.h>
#include <sandshark_msgs/ApplicationAbort.h>
#include <sandshark_msgs/Shutdown.h>
#include <sandshark_msgs/ObjectiveControlStatus.h>
#include <sandshark_msgs/DynamicRequestsStatus.h>

#define FS_MILLI_DIVISOR        1000
#define FS_CMD_TIMEOUT_MS       40

#define FS_SPI_VERSION_NUMBER   "SS_0.0"

using namespace std;
namespace bluefin {
namespace sandshark {

class FrontSeatDriver: public DriverBase {
  private:
    //Variables for the tcp server
    int _listenfd, _connfd, _n;
    struct sockaddr_in _servaddr, _cliaddr;
    socklen_t _clilen;
    char _mesg[256];
    fd_set _rfds;

    //Timeout for message responses
    struct timeval _message_timeout;

    //Interaction status variables
    bool _dynamicRequestsEnabled;
    bool _connected;
    bool _acksEnabled;
    typedef std::map<std::string, bool> logMap;
    logMap _logMap; //map to keep track of which log status messages are turned on

    //Various nav
    struct NavState _lastNavState;
    bool _receivedNav;
    float _headingRate, _rollRate, _pitchRate;
    float _eastVelocity, _northVelocity, _downVelocity;

    int _emptyRecvFromCount;

    std::string _missionStatus;
    std::string _extendedStatus;
    std::string _lastMissionStatus;
    std::string _lastExtendedStatus;

    ros::Rate * _rate;

    //subscribers (info going to the payload)
    ros::Subscriber _navSub;
    ros::Subscriber _velocitiesSub;
    ros::Subscriber _dcSpeedSub;
    ros::Subscriber _shutdownSub;
    ros::Subscriber _objectiveStatusSub;
    ros::Subscriber _dynamicRequestsSub;

    //publishers (for conveying payload requests and info)
    ros::Publisher _dcCmdPub;
    sandshark_msgs::DynamicControlCommand _dcCmdMsg;
    ros::Publisher _abortPub;
    sandshark_msgs::ApplicationAbort _abortMsg;
    ros::Publisher _psPub;
    sandshark_msgs::PayloadStatus _psMsg;

    ros::Time _lastRunTime;
    ros::Time _lastNavReceived;

    //Utility functions for sending messages to the payload
    bool sendMessageString(std::string messageString);
    bool sendBfMessage(bfMessage *b);

    //Messages we have to send to the payload
    bool sendNVGMessage();
    bool sendNVRMessage();
    bool sendCTLMessage();
    bool sendSHTMessage();
    bool sendMISMessage();
    bool sendVERMessage();
    bool sendACKIfNeeded(std::string h, std::string r, int c, int s, std::string d);

    //Utility functions for receiving payload messages and dispatching them appropriately
    void handleIncomingMessage(std::string inMessage);
    bool receiveMessage(struct timeval timeout);

    //Callbacks upon receipt of various payload messages.
    void handleRMB(std::string inMessage);
    void handleLOG(std::string inMessage);
    void handleABT(std::string inMessage);
    void handleSTS(std::string inMessage);
    void handleVER(std::string inMessage);

    //Fill in logMap with default startup values
    void fillLogMap();
    bool getLogStatus(std::string header);

    void cleanup();

  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();

  public:
    FrontSeatDriver() :
        DriverBase("FrontSeatDriver", "frontseatdriver") {
    }

    void msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg);
    void msgVelocitiesCallback(const sandshark_msgs::Velocities::ConstPtr & msg);
    void msgShutdownCallback(const sandshark_msgs::Shutdown::ConstPtr & msg);
    void msgDynamicRequestsCallback(const sandshark_msgs::DynamicRequestsStatus::ConstPtr & msg);
    void msgObjectiveStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg);

    void handleSleep();
};

void FrontSeatDriver::startupInitCallback() {

  _shutdownSub = _publicNode->subscribe("shutdown", 1, &FrontSeatDriver::msgShutdownCallback, this);
  _dynamicRequestsSub = _publicNode->subscribe("dynamic_requests_status", 1,
      &FrontSeatDriver::msgDynamicRequestsCallback, this);
  _objectiveStatusSub = _publicNode->subscribe("/objectivecontrol/objective_status", 1,
      &FrontSeatDriver::msgObjectiveStatusCallback, this);
  _navSub = _publicNode->subscribe("/navigation/navState", 1, &FrontSeatDriver::msgNavigationCallback, this);
  _velocitiesSub = _publicNode->subscribe("/navigation/velocities", 1, &FrontSeatDriver::msgVelocitiesCallback, this);

  _dcCmdPub = _publicNode->advertise<sandshark_msgs::DynamicControlCommand>("/dynamiccontrol/command", 10);
  _psPub = _publicNode->advertise<sandshark_msgs::PayloadStatus>("payload_status", 10);
  _abortPub = _publicNode->advertise<sandshark_msgs::ApplicationAbort>("/objectivecontrol/abort", 10);

  //running this gentleman at a faster rate will let us detect and deal with a client disconnection
  //quicker than at 10 Hz due to how doRun and receiveMessage are implemented.
  _rate = new ros::Rate(20);

  //default timeout for message responses
  int tsecs = (double) FS_CMD_TIMEOUT_MS / (double) FS_MILLI_DIVISOR;
  _message_timeout.tv_sec = tsecs;
  _message_timeout.tv_usec = ((double) FS_CMD_TIMEOUT_MS - tsecs * (double) FS_MILLI_DIVISOR)
      * (double) FS_MILLI_DIVISOR;
}

bool FrontSeatDriver::doInitialize() {
  ROS_DEBUG("Entering doInitialize");
  _connected = false;
  _receivedNav = false;
  _acksEnabled = false;
  _dynamicRequestsEnabled = false;
  _missionStatus = "Ready";
  _extendedStatus = "0";
  _lastMissionStatus = "Ready";
  _lastExtendedStatus = "0";
  _connected = false;
  _dynamicRequestsEnabled = false;
  _emptyRecvFromCount = 0;

  //2Chainz!!!
  fillLogMap();

  //set up the TCP socket for the backseat to connect to
  _listenfd = socket(AF_INET, SOCK_STREAM, 0);
  bzero(&_servaddr, sizeof(_servaddr));
  _servaddr.sin_family = AF_INET;
  _servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  _servaddr.sin_port = htons(29500);
  bind(_listenfd, (struct sockaddr *) &_servaddr, sizeof(_servaddr));
  listen(_listenfd, 5);

  //Block on a backseat client connection, don't enter running until then
  _clilen = sizeof(_cliaddr);
  _connfd = accept(_listenfd, (struct sockaddr *) &_cliaddr, &_clilen);
  _connected = true;
  sendVERMessage();
  ROS_DEBUG("Exiting doInitialize");
  return true;
}

void FrontSeatDriver::cleanup() {
  ROS_DEBUG("Entering Cleanup");
  //close everything socket related so that the connection can be re established next doInitialize
  close(_listenfd);
  close(_connfd);
  ROS_DEBUG("Exiting Cleanup");
}

bool FrontSeatDriver::doRun() {
  //receive a message if one is available.  Return the value from receiveMessage,
  //as it will return false on select returning -1, which indicates that the
  //connection has dropped.  This will make us reinitialize and wait for the
  //payload to reconnect
  return receiveMessage(_message_timeout);
}

void FrontSeatDriver::handleSleep() {
  _rate->sleep();
}

void FrontSeatDriver::msgShutdownCallback(const sandshark_msgs::Shutdown::ConstPtr & msg) {
  if (msg->shutdown) {
    sendSHTMessage();
  }
}

void FrontSeatDriver::msgObjectiveStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg) {
  //TODO: Need some different strings to match huxley
  if (msg->mission_running) {
    //_missionStatus = "Running";
    _missionStatus = " ";
  } else {
    if (msg->mission_loaded) {
      //_missionStatus = "Ready";
      _missionStatus = " ";
    } else {
      //_missionStatus = "Stopped";
      _missionStatus = " ";
    }
  }
  _extendedStatus = std::string("Waypoint: ") + str(boost::format("%d") % msg->current_waypoint);
  if (_lastMissionStatus.compare(_missionStatus) != 0 || _lastExtendedStatus.compare(_extendedStatus) != 0) {
    sendMISMessage();
  }
  _lastMissionStatus = _missionStatus;
  _lastExtendedStatus = _extendedStatus;
}

void FrontSeatDriver::msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg) {
  if (!_receivedNav) {
    _receivedNav = true;
    _headingRate = 0;
    _rollRate = 0;
    _pitchRate = 0;
    _lastNavReceived = ros::Time::now();
  } else {
    ros::Duration deltaT = ros::Time::now() - _lastNavReceived;
    //heading is 0 to 360 so no need to worry about wrapping
    _headingRate = (msg->bearing - _lastNavState.bearing) / deltaT.toSec();
    //roll goes from -180 to 180, so add 180 to deal with wrapping
    _rollRate = ((msg->roll + 180) - (_lastNavState.roll + 180)) / deltaT.toSec();
    //pitch goes from ~-80 to ~80, singularities are issues here, so
    //bound from -60 to 60 (seems reasonable, hail 2Chainz!!!!)
    //and then add 60 to take care of wrapping issues
    double lp = _lastNavState.pitch;
    if (lp > 60) {
      lp = 60;
    }
    if (lp < -60) {
      lp = -60;
    }
    double p = msg->pitch;
    if (p > 60) {
      p = 60;
    }
    if (p < -60) {
      p = -60;
    }
    _pitchRate = ((p + 60) - (lp + 60)) / deltaT.toSec();
    _lastNavReceived = ros::Time::now();
  }

  _lastNavState.latitude = msg->latitude;
  _lastNavState.longitude = msg->longitude;
  _lastNavState.quality = msg->quality;
  _lastNavState.altitude = msg->altitude;
  _lastNavState.depth = msg->depth;

  _lastNavState.bearing = msg->bearing;
  _lastNavState.roll = msg->roll;
  _lastNavState.pitch = msg->pitch;

  _lastNavState.computed_time = msg->computed_time;
  sendNVGMessage();
}

void FrontSeatDriver::msgVelocitiesCallback(const sandshark_msgs::Velocities::ConstPtr & msg) {
  _eastVelocity = msg->east_velocity;
  _northVelocity = msg->north_velocity;
  _downVelocity = msg->down_velocity;
  sendNVRMessage();
}

void FrontSeatDriver::msgDynamicRequestsCallback(const sandshark_msgs::DynamicRequestsStatus::ConstPtr & msg) {
  _dynamicRequestsEnabled = msg->dynamic_requests_enabled;
  printf("Sending CTL!!!!");

  sendMISMessage();
  usleep(20000);
  sendCTLMessage();
}

bool FrontSeatDriver::sendMessageString(std::string messageString) {
  messageString = messageString.append("\n");
  int sentBytes = sendto(_connfd, messageString.c_str(), messageString.length(), 0, (struct sockaddr *) &_cliaddr,
      sizeof(_cliaddr));
  if (sentBytes != (int) messageString.length()) {
    return false;
  }
  return true;
}

bool FrontSeatDriver::sendBfMessage(bfMessage *b) {
  std::string messageString;
  b->unparse(messageString);
  printf("Sending %s\n", messageString.c_str());
  if (!sendMessageString(messageString)) {
    deviceTimeoutWarning("sendBfMessage sent the incorrect number of bytes");
    return false;
  }
  return true;
}

bool FrontSeatDriver::sendNVGMessage() {
  if (!getLogStatus("NVG")) {
    return true;
  }
  nvgMessage n;

  n.setLatitudeAndHemisphere(_lastNavState.latitude);
  n.setLongitudeAndHemisphere(_lastNavState.longitude);
  n.setPositionQuality(_lastNavState.quality);
  n.setAltitude(_lastNavState.altitude);
  n.setDepth(_lastNavState.depth); //2Chainz
  n.setHeading(_lastNavState.bearing);
  n.setRoll(_lastNavState.roll);
  n.setPitch(_lastNavState.pitch);
  n.setComputedTime(_lastNavState.computed_time);

  return sendBfMessage((bfMessage *) &n);
}

bool FrontSeatDriver::sendNVRMessage() {
  if (!getLogStatus("NVR")) {
    return true;
  }
  nvrMessage n;
  n.setEastVelocity(_eastVelocity);
  n.setWestVelocity(_northVelocity);
  n.setDownVelocity(_downVelocity);
  n.setHeadingRate(_headingRate);
  n.setRollRate(_rollRate);
  n.setPitchRate(_pitchRate);
  return sendBfMessage((bfMessage *) &n);
}

bool FrontSeatDriver::sendCTLMessage() {
  if (!getLogStatus("CTL")) {
    return true;
  }
  ctlMessage c;
  c.setControlState(_dynamicRequestsEnabled ? 1 : 0);
  return sendBfMessage((bfMessage *) &c);
}

bool FrontSeatDriver::sendSHTMessage() {
  //shutdown has no fields
  if (!getLogStatus("SHT")) {
    return true;
  }
  shtMessage s;
  return sendBfMessage((bfMessage *) &s);
}

bool FrontSeatDriver::sendMISMessage() {
  if (!getLogStatus("MIS")) {
    return true;
  }
  misMessage m;
  m.setDiveFile("Something.bm2");  //no notion of a dive file right now
  //TODO: talk to MIT and see if this it necessary
  m.setMissionStatus(_missionStatus);
  m.setAdditionalStatus(_extendedStatus);

  return sendBfMessage((bfMessage *) &m);
}

bool FrontSeatDriver::sendVERMessage() {
  if (!getLogStatus("VER")) {
    return true;
  }
  verMessageF v;
  v.setVersionString(FS_SPI_VERSION_NUMBER);
  return sendBfMessage((bfMessage *) &v);
}

bool FrontSeatDriver::sendACKIfNeeded(std::string h, std::string r, int c, int s, std::string d) {
  if (!_acksEnabled) {
    return true;
  }
  ackMessage a;
  a.setAckHeader(h);
  a.setReceivedTimestamp(r);
  a.setCommandID(c);
  a.setAckStatusCode(s);
  a.setDescription(d);
  return sendBfMessage((bfMessage *) &a);
}

//read in exactly one message
bool FrontSeatDriver::receiveMessage(struct timeval timeout) {
  //local copy of the timeval so that we can use it with select
  struct timeval tval_recvdelay;
  tval_recvdelay.tv_usec = timeout.tv_usec;
  tval_recvdelay.tv_sec = timeout.tv_sec;

  //clear the set and then add the rfds descriptor to it
  FD_ZERO(&_rfds);
  FD_SET(_connfd, &_rfds);
  FD_SET(STDIN_FILENO, &_rfds);
  //ROS_INFO("Receiving time out = %d : %d", (int) tval_recvdelay.tv_sec, (int) tval_recvdelay.tv_usec);

  //use select to block until we receive a response
  int retval = select(_connfd + 1, &_rfds, NULL, NULL, &tval_recvdelay);

  //see what we get from the select
  if (retval == -1) {
    //This indicates that the client has disconnected.  Return false, which should get us back
    return false;
  } else if (retval) {
    printf("Got probably not garbage\n");
    //TODO - Make this not terrible. As it stands, it could easily receive 1.X messages rather than 1
    //need to read into a buffer where we can check for the $ to indicate new messages, or do it here
    _n = recvfrom(_connfd, _mesg, 256, 0, (struct sockaddr *) &_cliaddr, &_clilen);
    _mesg[_n] = 0;
    //if the client disconnects, we will receive a bunch of empty strings
    if (_n == 0) {
      _emptyRecvFromCount++;
    }
    if (_emptyRecvFromCount >= 5) {
      return false;
    }
    std::string inMessage(_mesg);
    handleIncomingMessage(inMessage);
    return true;
  } else {
    //simple timeout, no big deal, maybe we'll get a message next time.
    //try not to think about the cruel silent treatment that the payload
    //is giving us at the moment.  Cry as the crippling sting of loneliness engulfs us
    return true;
  }
}

void FrontSeatDriver::handleIncomingMessage(std::string inMessage) {
  printf("Got incoming message!! %s\n", inMessage.c_str());
  if (boost::algorithm::contains(inMessage, "RMB")) {
    handleRMB(inMessage);
  } else if (boost::algorithm::contains(inMessage, "LOG")) {
    handleLOG(inMessage);
  } else if (boost::algorithm::contains(inMessage, "ABT")) {
    handleABT(inMessage);
  } else if (boost::algorithm::contains(inMessage, "STS")) {
    handleSTS(inMessage);
  } else if (boost::algorithm::contains(inMessage, "VER")) {
    handleVER(inMessage);
  } else {
    ROS_ERROR("Unidentified message : \"%s\" received by FrontSeatDriver\n", inMessage.c_str());
    //TODO: what do we send here for a header
    sendACKIfNeeded("???", "?????", -1, 0,
        str(boost::format("Received Unidentified message \"%s\"") % inMessage.c_str()));
  }
}

void FrontSeatDriver::handleRMB(std::string inMessage) {
  printf("got rmb\n");
  rmbMessage rmb;
  if (rmb.parse(inMessage)) {
    sendACKIfNeeded(rmb.getHeader(), rmb.getTime(), -1, 2, "Successfully processed RMB message");
    //fill out a dc message with message info and fill it out
    //This is dangerous, and commented out for now
    //ACK still goes out, but vehicle will not move
/*    _dcCmdMsg.vertical_mode = rmb.getVerticalMode();
    _dcCmdMsg.vertical_desired = rmb.getVertical();
    _dcCmdMsg.horizontal_mode = rmb.getHorizontalMode();
    _dcCmdMsg.horizontal_desired = rmb.getHorizontal();
    if(rmb.getTranslationalMode() == 1){
      _dcCmdMsg.speed_desired = 800 * rmb.getTranslational(); //hack until we decide
      //and characterize actual prop rpm vs speed over ground
    } else {
      _dcCmdMsg.speed_desired = rmb.getTranslational(); //only can do rpm
    }
    _dcCmdPub.publish(_dcCmdMsg);*/  
  } else {
    sendACKIfNeeded(rmb.getHeader(), rmb.getTime(), -1, 1, "Failed to process RMB message");
  }
}

void FrontSeatDriver::handleLOG(std::string inMessage) {
  printf("got log\n");
  logMessage log;
  if (log.parse(inMessage)) {
    printf("parsed correctly");
    printf("header\n");
    printf("%s\n", log.getHeader().c_str());
    printf("msgheader\n");
    printf("%s\n", log.getMsgHeader().c_str());
    printf("onoff\n");
    printf("%s\n", log.getOnOff().c_str());

    sendACKIfNeeded(log.getHeader(), log.getTime(), -1, 2, "Successfully processed LOG message");
    //if we have an ALL either set the whole map to true or false

    if (log.getMsgHeader().compare("ALL") == 0) {
      printf("got all on\n");
      std::map<std::string, bool>::iterator iter;
      for (iter = _logMap.begin(); iter != _logMap.end(); ++iter) {
        iter->second = ((log.getOnOff().compare("ON") == 0) ? true : false);
      }
    } else if (log.getMsgHeader().compare("ACK") == 0) {
      //ACK just means turn all message acks on
      printf("got log ack on\n");
      _acksEnabled = ((log.getOnOff().compare("ON") == 0) ? true : false);
    } else {
      printf("not ack or anything %s %s\n", log.getMsgHeader().c_str(), log.getOnOff().c_str());
      _logMap[log.getMsgHeader()] = ((log.getOnOff().compare("ON") == 0) ? true : false);
    }
  } else {
    sendACKIfNeeded(log.getHeader(), log.getTime(), -1, 1, "Failed to process LOG message");
    printf("failed to parse");
  }
}

void FrontSeatDriver::handleABT(std::string inMessage) {
  printf("got abt\n");
  abtMessage abt;
  if (abt.parse(inMessage)) {
    sendACKIfNeeded(abt.getHeader(), abt.getTime(), -1, 2, "Successfully processed ABT message");
    //_abortMsg.abort = ((abt.getAbortCode() == 1) ? true : false);
    //doesn't matter what the abort code is, we must still abort
    _abortMsg.abort = true;
    _abortMsg.reason = abt.getAbortMessage();
    _abortPub.publish(_abortMsg);
  } else {
    sendACKIfNeeded(abt.getHeader(), abt.getTime(), -1, 1, "Failed to process ABT message");
  }
}

void FrontSeatDriver::handleSTS(std::string inMessage) {
  printf("got sts\n");
  stsMessage sts;
  if (sts.parse(inMessage)) {
    sendACKIfNeeded(sts.getHeader(), sts.getTime(), -1, 2, "Successfully processed STS message");
    _psMsg.status_code = sts.getStatusFlag();
    _psMsg.status_description = sts.getStatusText();
    _psPub.publish(_psMsg);
  } else {
    sendACKIfNeeded(sts.getHeader(), sts.getTime(), -1, 1, "Failed to process STS message");
  }
}

void FrontSeatDriver::handleVER(std::string inMessage) {
  verMessageP ver;
  if (ver.parse(inMessage)) {
    sendACKIfNeeded(ver.getHeader(), ver.getTime(), -1, 2, "Successfully processed VER message");
    ROS_INFO("Backseat connection is using interface version: %s", ver.getVersionString().c_str());
  } else {
    sendACKIfNeeded(ver.getHeader(), ver.getTime(), -1, 1, "Failed to process VER message");
  }
}

void FrontSeatDriver::fillLogMap() {
  _logMap["CTL"] = true;
  _logMap["NVG"] = false;
  _logMap["NVR"] = false;
  _logMap["SHT"] = false;
  _logMap["MIS"] = true;
  _logMap["VER"] = true; //because we will always send at the beginning, son
}

bool FrontSeatDriver::getLogStatus(std::string header) {
  logMap::iterator iter;
  for (iter = _logMap.begin(); iter != _logMap.end(); ++iter) {
    if (header.compare(iter->first) == 0) {
      return iter->second;
    } else {
    }
  }
  return false;
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::FrontSeatDriver ad;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) ad, argc, argv);
}
