//#include <android/sensor.h>
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
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <Eigen>
#include <mavlink.h>
#include <adapt.h>

#include <ros/ros.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/MavlinkWaypointList.h>
#include <sandshark_msgs/MavlinkPolygon.h>
#include <sandshark_msgs/ObjectiveCommand.h>
#include <sandshark_msgs/ObjectiveControlStatus.h>
#include <sandshark_msgs/MotionRaw.h>
#include <sandshark_msgs/DynamicControlGains.h>
#include <sandshark_msgs/HealthValues.h>
#include <sandshark_msgs/UpdateDCGain.h>
#include <sandshark_msgs/UpdateHealthVal.h>
#include <sandshark_msgs/AccelCalib.h>
#include <sandshark_msgs/MagCalib.h>
#include <sandshark_msgs/GasGauge.h>
#include "sandshark_msgs/adapt_payload_data.h"

#include "ellipsoidFit.h"

// Silence warning re: MAVLINK_ALIGNED_FIELDS redefinition
#ifdef MAVLINK_ALIGNED_FIELDS
#undef MAVLINK_ALIGNED_FIELDS
#endif

#define MAVLINK_ALIGNED_FIELDS false
#include <mavlink.h>

using namespace std;
using namespace Eigen;
namespace bluefin {
namespace sandshark {

class MavLinkDriver: public DriverBase {
  private:

    ros::Subscriber _navSub;
    ros::Subscriber _payloadSub;
    ros::Subscriber _objStatusSub;
    ros::Subscriber _waypointListSub;
    ros::Subscriber _rawSensorSub;
    ros::Subscriber _dcGainSub;
    ros::Subscriber _healthValsSub;
    ros::Subscriber _gasGaugeSub;

    ros::ServiceClient _accelCalibClient;
    sandshark_msgs::AccelCalib _accelCalibSrv;

    ros::ServiceClient _magCalibClient;
    sandshark_msgs::MagCalib _magCalibSrv;

    ros::ServiceClient _dcUpdateGainClient;
    sandshark_msgs::UpdateDCGain _dcUpdateGainSrv;

    ros::ServiceClient _healthUpdateValClient;
    sandshark_msgs::UpdateHealthVal _healthUpdateValSrv;

    ros::Publisher _recvPointListPub;
    sandshark_msgs::MavlinkWaypointList _recvPointMsg;

    ros::Publisher _recvPolygonPub;
    sandshark_msgs::MavlinkPolygon _recvPolygonMsg;

    ros::Publisher _objCommandPub;
    sandshark_msgs::ObjectiveCommand _objCmdMsg;

    bool _haveWaypointList;
    struct Waypoint {
        uint8_t coord_frame;
        uint16_t command;
        double param1;
        double param2;
        double param3;
        double param4;
        double param5;
        double param6;
        double param7;
        bool auto_continue;
    };

    int _currentWaypoint;
    std::vector<struct Waypoint> _currentList;
    std::vector<struct Waypoint> _receivedList;

    int _cachedCurrent;
    std::vector<struct Waypoint> _cachedList;

    string _sendIPAddress;
    string _secondarySendIPAddress;
    int _systemID;
    bool _verboseCalibration;
    int _gcsID;
    int _socket;
    struct sockaddr_in _groundControlAddr;
    struct sockaddr_in _secondaryControlAddr;
    struct sockaddr_in _recvAddr;
    mavlink_message_t _mavMessage;
    enum {
      BUFFER_LENGTH = 2048
    };
    uint8_t _sendRecvBuf[BUFFER_LENGTH];

    uint64_t microsSinceEpoch();

    double _latMeasured;
    double _lngMeasured;
    double _speedMeasured;
    double _depthMeasured;
    double _altitudeMeasured;
    double _pitchMeasured;
    double _rollMeasured;
    double _bearingMeasured;
    int32_t _payloadLat;
    int32_t _payloadLon;
    double _payloadFlour;
    ros::Time _lastMeasuredTime;
    ros::Time _lastMeasuredSentTime;
    ros::Time _lastMeasuredPayloadTime;
    ros::Time _lastMeasuredSentPayloadTime;
    ros::Time _lastStatusTime;

    bool _objStatHaveNav;
    bool _objStatRunningMission;
    ros::Time _lastObjStatusTime;

    int _lookingForWaypoints;
    int _currentLookingForPoint;
    ros::Time _lookingForStartTime;

    std::map<std::string, double> _paramValues;
    std::map<std::string, enum DynamicControlGainUpdate> _paramDCIDs;
    std::map<std::string, enum HealthValueUpdate> _paramHealthIDs;

    double _battVoltage;
    double _battAverageCurrent;
    int16_t _battPercentRemaining;

    char _statMsg[50];

    enum {
      MAG_CAL_OFF, MAG_CAL_HOLD, MAG_CAL_SAMPLING, MAG_CAL_FIT
    } _calibrateMag;

    vector<Vector3d> _magSamples;
    bool _sampleMag;

    enum {
      ACCEL_CAL_OFF,
      ACCEL_CAL_FLAT,
      ACCEL_CAL_RIGHT,
      ACCEL_CAL_TOP,
      ACCEL_CAL_LEFT,
      ACCEL_CAL_TAIL,
      ACCEL_CAL_NOSE,
      ACCEL_CAL_FLAT2,
    } _calibrateAccel;

    vector<Vector3d> _accelSamples;
    bool _sampleAccel;
    bool _waitingForCalibrationAck;
    ros::Time _calibrationStartTime;

    void handleCalibration();

    bool sendData(uint8_t* buf, int len);

    void sendWaypointRequest(int wpID);
    void sendMissionACK(int type = MAV_MISSION_ACCEPTED);
    void sendParams();
    void sendCommandACK(int command, int result);
    void sendStatusText(const char * text, int severity = MAV_SEVERITY_ALERT);

    ros::Rate * _rate;
    void cleanup();

    bool string_contains(const string & text, const string substring);
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    MavLinkDriver() :
        DriverBase("MavLinkDriver", "mavlinkdriver") {
    }

    void msgCurrentWaypointCallback(const sandshark_msgs::MavlinkWaypointList::ConstPtr & msg);
    void msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg);
    void msgPayloadCallback(const sandshark_msgs::adapt_payload_data::ConstPtr & msg);
    void msgObjStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg);
    void msgRawSensorCallback(const sandshark_msgs::MotionRaw::ConstPtr & msg);
    void msgCurrentDCGainsCallback(const sandshark_msgs::DynamicControlGains::ConstPtr & msg);
    void msgCurrentHealthValsCallback(const sandshark_msgs::HealthValues::ConstPtr & msg);
    void msgGasGaugeCallback(const sandshark_msgs::GasGauge::ConstPtr & msg);

    void handleSleep();
    void handleShutdown() {
      cleanup();
    }
};

void MavLinkDriver::msgCurrentWaypointCallback(const sandshark_msgs::MavlinkWaypointList::ConstPtr & msg) {
  if (msg->number_of_points != msg->command.size()) {
    return;
  }

  _currentList.clear();
  struct Waypoint waypoint;
  for (unsigned int index = 0; index < msg->number_of_points; ++index) {
    waypoint.coord_frame = msg->coord_frame[index];
    waypoint.command = msg->command[index];
    waypoint.param1 = msg->param1[index];
    waypoint.param2 = msg->param2[index];
    waypoint.param3 = msg->param3[index];
    waypoint.param4 = msg->param4[index];
    waypoint.param5 = msg->param5[index];
    waypoint.param6 = msg->param6[index];
    waypoint.param7 = msg->param7[index];
    waypoint.auto_continue = msg->auto_continue[index];
    _currentList.push_back(waypoint);
  }

  _currentWaypoint = msg->current_point;
}

void MavLinkDriver::msgObjStatusCallback(const sandshark_msgs::ObjectiveControlStatus::ConstPtr & msg) {
  _objStatHaveNav = msg->have_nav;
  _objStatRunningMission = msg->mission_running;
  _lastObjStatusTime = ros::Time::now();
}

void MavLinkDriver::msgNavigationCallback(const sandshark_msgs::Navigation::ConstPtr & msg) {
  _latMeasured = msg->latitude;
  _lngMeasured = msg->longitude;
  _speedMeasured = msg->speed;
  _depthMeasured = msg->depth;
  _altitudeMeasured = msg->altitude;
  _pitchMeasured = msg->pitch;
  _rollMeasured = msg->roll;
  _bearingMeasured = msg->bearing;
  _lastMeasuredTime = ros::Time::now();
}

void MavLinkDriver::msgPayloadCallback(const sandshark_msgs::adapt_payload_data::ConstPtr & msg) {
  _payloadLat = (int) (msg->lat * 100000);
  _payloadLon = (int) (msg->lon * 100000);
  _payloadFlour = msg->fluor;
  _paramValues["Flourescence"] = _payloadFlour;
  _lastMeasuredPayloadTime = ros::Time::now();
}

void MavLinkDriver::msgRawSensorCallback(const sandshark_msgs::MotionRaw::ConstPtr & msg) {
  if (_sampleAccel) {
    Vector3d sample;
    sample[0] = msg->accel_x;
    sample[1] = msg->accel_y;
    sample[2] = msg->accel_z;
    _accelSamples.push_back(sample);
    if (_verboseCalibration) {
      ROS_INFO("%d acceleration calibration samples", (int) _accelSamples.size());
    }

    snprintf(_statMsg, sizeof(_statMsg), "Sampling ACCEL -- Mode %d, size %d", _calibrateAccel,
        (int) _accelSamples.size());
    _statMsg[sizeof(_statMsg) - 1] = '\0';
    sendStatusText(_statMsg);
  }

  if (_sampleMag) {
    Vector3d sample;
    sample[0] = msg->mag_x;
    sample[1] = msg->mag_y;
    sample[2] = msg->mag_z;
    _magSamples.push_back(sample);

    snprintf(_statMsg, sizeof(_statMsg), "Sampling MAG -- Mode %d, size %d", _calibrateAccel, (int) _magSamples.size());
    _statMsg[sizeof(_statMsg) - 1] = '\0';
    sendStatusText(_statMsg);
  }

}

void MavLinkDriver::msgCurrentDCGainsCallback(const sandshark_msgs::DynamicControlGains::ConstPtr & msg) {
  _paramValues["dctrl_heading_p"] = msg->headingP;
  _paramValues["dctrl_heading_i"] = msg->headingI;
  _paramValues["dctrl_heading_d"] = msg->headingD;

  _paramValues["dctrl_depth_p"] = msg->depthP;
  _paramValues["dctrl_depth_i"] = msg->depthI;
  _paramValues["dctrl_depth_d"] = msg->depthD;

  _paramValues["dctrl_pitch_p"] = msg->pitchP;
  _paramValues["dctrl_pitch_i"] = msg->pitchI;
  _paramValues["dctrl_pitch_d"] = msg->pitchD;

  _paramValues["dctrl_rpm_to_ms"] = msg->rpm_to_speed;
}

void MavLinkDriver::msgCurrentHealthValsCallback(const sandshark_msgs::HealthValues::ConstPtr & msg) {
  _paramValues["health_max_temp"] = msg->max_temp;
  _paramValues["health_max_pres"] = msg->max_pressure;
  _paramValues["health_min_pres"] = msg->min_pressure;
  _paramValues["health_max_dept"] = msg->max_depth;
  _paramValues["health_min_alt"] = msg->max_altitude;
  _paramValues["health_min_batt"] = msg->min_battery;
  _paramValues["health_max_sec"] = msg->max_mission_seconds;
}

void MavLinkDriver::msgGasGaugeCallback(const sandshark_msgs::GasGauge::ConstPtr & msg) {
  _battVoltage = msg->voltage;
  _battAverageCurrent = msg->average_current;
  _battPercentRemaining = msg->percent_remaining;
}

void MavLinkDriver::startupInitCallback() {

  ROS_DEBUG("Entering startupInitCallback");

  _privateNode->param("sendIPAddress", _sendIPAddress, std::string("127.0.0.1"));
  _privateNode->param("secondarySendIPAddress", _secondarySendIPAddress, std::string(""));

  _privateNode->param("systemID", _systemID, (int) 0);
  _privateNode->param("verboseCalibration", _verboseCalibration, true);
  _privateNode->param("gcsID", _gcsID, (int) 254);

  ROS_INFO("sendIP = %s", _sendIPAddress.c_str());
  ROS_INFO("secondarySendIP = %s", _secondarySendIPAddress.c_str());
  _navSub = _publicNode->subscribe("/navigation/navState", 1, &MavLinkDriver::msgNavigationCallback, this);
  _payloadSub = _publicNode->subscribe("/payload/payload_msg", 1, &MavLinkDriver::msgPayloadCallback, this);
  _objStatusSub = _publicNode->subscribe("/objectivecontrol/objective_status", 1, &MavLinkDriver::msgObjStatusCallback,
      this);
  _waypointListSub = _publicNode->subscribe("/objectivecontrol/waypoint_list", 1,
      &MavLinkDriver::msgCurrentWaypointCallback, this);
  _rawSensorSub = _publicNode->subscribe("/motion/raw_sensors", 1, &MavLinkDriver::msgRawSensorCallback, this);
  _dcGainSub = _publicNode->subscribe("/dynamiccontrol/current_gains", 1, &MavLinkDriver::msgCurrentDCGainsCallback,
      this);
  _healthValsSub = _publicNode->subscribe("/health/current_values", 1, &MavLinkDriver::msgCurrentHealthValsCallback,
      this);
  _gasGaugeSub = _publicNode->subscribe("/gasgauge/gasgauge", 1, &MavLinkDriver::msgGasGaugeCallback, this);

  _accelCalibClient = _publicNode->serviceClient<sandshark_msgs::AccelCalib>("/motion/calibrateAccel");
  _magCalibClient = _publicNode->serviceClient<sandshark_msgs::MagCalib>("/motion/calibrateMag");
  _dcUpdateGainClient = _publicNode->serviceClient<sandshark_msgs::UpdateDCGain>("/dynamiccontrol/update_gain");
  _healthUpdateValClient = _publicNode->serviceClient<sandshark_msgs::UpdateHealthVal>("/health/update_value");

  _recvPointListPub = _publicNode->advertise<sandshark_msgs::MavlinkWaypointList>(
      "/objectivecontrol/command/received_waypoint_list", 1);
  _recvPolygonPub = _publicNode->advertise<sandshark_msgs::MavlinkPolygon>("/objectivecontrol/command/received_polygon",
      1);
  _objCommandPub = _publicNode->advertise<sandshark_msgs::ObjectiveCommand>(
      "/objectivecontrol/command/objective_command", 1);

  _rate = new ros::Rate(10);
}

bool MavLinkDriver::doInitialize() {

  ROS_DEBUG("Entering doInitialize");

  //do a cleanup
  cleanup();

  _socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

  memset(&_recvAddr, 0, sizeof(_recvAddr));
  _recvAddr.sin_family = AF_INET;
  _recvAddr.sin_addr.s_addr = INADDR_ANY;
  _recvAddr.sin_port = htons(14551);

  /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
  if (-1 == bind(_socket, (struct sockaddr *) &_recvAddr, sizeof(struct sockaddr))) {
    setErrorMessage("error: recv bind failed");
    cleanup();
    return false;
  }

  /* Attempt to make it non blocking */
  if (fcntl(_socket, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    char errorbuf[128];
    snprintf(&errorbuf[0], sizeof(errorbuf), "error setting nonblocking: %s\n", strerror(errno));
    errorbuf[sizeof(errorbuf) - 1] = '\0';
    setErrorMessage(errorbuf);
    return false;
  }

  memset(&_groundControlAddr, 0, sizeof(_groundControlAddr));
  _groundControlAddr.sin_family = AF_INET;
  _groundControlAddr.sin_addr.s_addr = inet_addr(_sendIPAddress.c_str());
  _groundControlAddr.sin_port = htons(14550);

  if (_secondarySendIPAddress != "") {
    memset(&_secondaryControlAddr, 0, sizeof(_secondaryControlAddr));
    _secondaryControlAddr.sin_family = AF_INET;
    _secondaryControlAddr.sin_addr.s_addr = inet_addr(_secondarySendIPAddress.c_str());
    _secondaryControlAddr.sin_port = htons(14550);
  }

  _lastStatusTime = ros::Time(0);
  _lastMeasuredTime = ros::Time(0);
  _lastMeasuredSentTime = ros::Time(0);
  _lastMeasuredPayloadTime = ros::Time(0);
  _lastMeasuredSentPayloadTime = ros::Time(0);

  _lookingForWaypoints = 0;
  _currentLookingForPoint = 0;
  _lookingForStartTime = ros::Time(0);

  _cachedCurrent = -1;

  _calibrateAccel = ACCEL_CAL_OFF;
  _calibrateMag = MAG_CAL_OFF;
  _waitingForCalibrationAck = false;
  _calibrationStartTime = ros::Time(0);
  _sampleAccel = false;
  _sampleMag = false;

  // Dynamic Control parameters
  _paramValues["dctrl_heading_p"] = 0.0;
  _paramDCIDs["dctrl_heading_p"] = GU_HEADING_P;

  _paramValues["dctrl_heading_i"] = 0.0;
  _paramDCIDs["dctrl_heading_i"] = GU_HEADING_I;

  _paramValues["dctrl_heading_d"] = 0.0;
  _paramDCIDs["dctrl_heading_d"] = GU_HEADING_D;

  _paramValues["dctrl_depth_p"] = 0.0;
  _paramDCIDs["dctrl_depth_p"] = GU_DEPTH_P;

  _paramValues["dctrl_depth_i"] = 0.0;
  _paramDCIDs["dctrl_depth_i"] = GU_DEPTH_I;

  _paramValues["dctrl_depth_d"] = 0.0;
  _paramDCIDs["dctrl_depth_d"] = GU_DEPTH_D;

  _paramValues["dctrl_pitch_p"] = 0.0;
  _paramDCIDs["dctrl_pitch_p"] = GU_PITCH_P;

  _paramValues["dctrl_pitch_i"] = 0.0;
  _paramDCIDs["dctrl_pitch_i"] = GU_PITCH_I;

  _paramValues["dctrl_pitch_d"] = 0.0;
  _paramDCIDs["dctrl_pitch_d"] = GU_PITCH_D;

  _paramValues["dctrl_rpm_to_ms"] = 0.0;
  _paramDCIDs["dctrl_rpm_to_ms"] = GU_RPMTOSPEED;

  _paramValues["Fluorescence"] = 0.0;
  _paramDCIDs["Fluorescence"] = GU_FLUOR;

  // Health parameters, component 2
  _paramValues["health_max_temp"] = 0.0;
  _paramHealthIDs["health_max_temp"] = HV_MAX_TEMP;

  _paramValues["health_max_pres"] = 0.0;
  _paramHealthIDs["health_max_pres"] = HV_MAX_PRESSURE;

  _paramValues["health_min_pres"] = 0.0;
  _paramHealthIDs["health_min_pres"] = HV_MIN_PRESSURE;

  _paramValues["health_max_dept"] = 0.0;
  _paramHealthIDs["health_max_dept"] = HV_MAX_DEPTH;

  _paramValues["health_min_alt"] = 0.0;
  _paramHealthIDs["health_min_alt"] = HV_MIN_ALTITUDE;

  _paramValues["health_min_batt"] = 0.0;
  _paramHealthIDs["health_min_batt"] = HV_MIN_BATTERY;

  _paramValues["health_max_sec"] = 0.0;
  _paramHealthIDs["health_max_sec"] = HV_MAX_MISSION_SECONDS;

  _battVoltage = 0.00;
  _battAverageCurrent = -1.0;
  _battPercentRemaining = -1;

  return true;
}

void MavLinkDriver::cleanup() {
  ROS_DEBUG("Cleaning Up");
  close(_socket);
}

bool MavLinkDriver::sendData(uint8_t* buf, int len) {
  bool success = true;
  int bytes_sent;
  bytes_sent = sendto(_socket, buf, len, 0, (struct sockaddr*) &_groundControlAddr, sizeof(struct sockaddr_in));
  if (bytes_sent < 0) {
    ROS_WARN_THROTTLE(30, "Bad Sendto groundcontrol errno %d:%s",
    errno, strerror( errno));
    success = false;
  }

  if (_secondarySendIPAddress != "") {
    bytes_sent = sendto(_socket, buf, len, 0, (struct sockaddr*) &_secondaryControlAddr, sizeof(struct sockaddr_in));
    if (bytes_sent < 0) {
      ROS_WARN_THROTTLE(30, "Bad Sendto secondary control errno %d:%s",
      errno, strerror( errno));
      success = false;
    }
  }

  return success;
}

void MavLinkDriver::sendWaypointRequest(int wpID) {
  mavlink_msg_mission_request_pack(_systemID, MAV_COMP_ID_ALL, &_mavMessage, _gcsID, MAV_COMP_ID_ALL, wpID);
  int len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
  bool result = sendData(_sendRecvBuf, len);
  if (!result) {
    ROS_WARN_THROTTLE(30, "Bad send for MavLink MissionRequest wpID %d", wpID);
  }
}

void MavLinkDriver::sendMissionACK(int type) {
  ROS_INFO("Send Mission ACK type %d", type);
  mavlink_msg_mission_ack_pack(_systemID, MAV_COMP_ID_ALL, &_mavMessage, _gcsID, 0, type);
  int len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
  bool result = sendData(_sendRecvBuf, len);
  if (!result) {
    ROS_WARN_THROTTLE(30, "Bad send for MavLink Mission Clear All Ack response");
  }
}

void MavLinkDriver::sendParams() {
  int index = 0;
  int len;
  bool success;

  ROS_INFO("Starting to send params");

  for (std::map<std::string, double>::const_iterator pIter = _paramValues.begin(); pIter != _paramValues.end();
      ++pIter) {
    mavlink_msg_param_value_pack(_systemID, 200, &_mavMessage, pIter->first.c_str(), pIter->second,
        MAV_PARAM_TYPE_REAL32, _paramValues.size(), index);
    len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
    success = sendData(_sendRecvBuf, len);
    if (!success) {
      ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Param %d", index);
    }
    index++;
    usleep(1000);
  }
}

void MavLinkDriver::sendCommandACK(int command, int result) {
  ROS_INFO("Send Command ACK command %d result %d", command, result);
  mavlink_msg_command_ack_pack(_systemID, MAV_COMP_ID_ALL, &_mavMessage, command, result);
  int len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
  bool success = sendData(_sendRecvBuf, len);
  if (!success) {
    ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Mission Command Ack response");
  }
}

void MavLinkDriver::sendStatusText(const char * text, int severity) {
  ROS_INFO("Sending Severity %d Stat Msg %s", severity, text);
  mavlink_msg_statustext_pack(_systemID, 200, &_mavMessage, severity, text);
  int len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
  bool result = sendData(_sendRecvBuf, len);
  if (!result) {
    ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Send Text response");
  }
}

bool MavLinkDriver::string_contains(const string & text, const string substring) {
  return text.find(substring) != string::npos;
}

bool MavLinkDriver::doRun() {
  static uint32_t sensorMask = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL
      | MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE | MAV_SYS_STATUS_SENSOR_GPS;
  uint16_t len;
  bool success;

  handleCalibration();

  ros::Time now = ros::Time::now();
  if ((now - _lastStatusTime) > ros::Duration(1.0)) {
    uint8_t state = MAV_STATE_UNINIT;
    if ((now - _lastObjStatusTime) < ros::Duration(2.0)) {
      if (_objStatHaveNav) {
        if (_objStatRunningMission) {
          state = MAV_STATE_ACTIVE;
        } else {
          state = MAV_STATE_STANDBY;
        }
      } else {
        state = MAV_STATE_BOOT;
      }
    }
    /*Send Heartbeat */
    mavlink_msg_heartbeat_pack(_systemID, 200, &_mavMessage, MAV_TYPE_SUBMARINE,
    //                                    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY/*MAV_AUTOPILOT_GENERIC*/,
        /*MAV_AUTOPILOT_PX4,*/
        MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, state);
    len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
    success = sendData(_sendRecvBuf, len);
    if (!success) {
      ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Heartbeat");
    }
    /* Send Status */
    mavlink_msg_sys_status_pack(_systemID, 200, &_mavMessage, sensorMask, sensorMask, sensorMask,
        0 /* do we know load from currentsense * voltage?*/, (uint16_t) floor(_battVoltage * 1000.0),
        (int16_t) floor(_battAverageCurrent * 1000.0), (int8_t) _battPercentRemaining, -1, 0, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
    success = sendData(_sendRecvBuf, len);
    if (!success) {
      ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Status errno");
    }

    //battery
    /*mavlink_msg_battery_status_pack(_systemID, uint8_t component_id, mavlink_message_t* msg,
     uint8_t accu_id, uint16_t voltage_cell_1, uint16_t voltage_cell_2, uint16_t voltage_cell_3, uint16_t voltage_cell_4, uint16_t voltage_cell_5, uint16_t voltage_cell_6, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining)
     */

    //send vehicle data
    if ((_lastMeasuredTime > ros::Time(0.0)) && (_lastMeasuredTime - _lastMeasuredSentTime) > ros::Duration(0.1)) {
      mavlink_msg_global_position_int_pack(_systemID, 200, &_mavMessage, microsSinceEpoch(),
          (int32_t) (_latMeasured * 1e7), (int32_t) (_lngMeasured * 1e7), (int32_t) (-1.0 * _depthMeasured * 1e3),
          (int32_t) (-1.0 * _depthMeasured * 1e3), (int16_t) (_speedMeasured * 1e2), 0, 0,
          (uint16_t) (_bearingMeasured * 1e2));
      len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
      success = sendData(_sendRecvBuf, len);
      if (!success) {
        ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Global Pos");
      }

      /* Send attitude */
      mavlink_msg_attitude_pack(_systemID, 200, &_mavMessage, microsSinceEpoch(), _rollMeasured / 180.0 * M_PI,
          _pitchMeasured / 180.0 * M_PI, _bearingMeasured / 180.0 * M_PI, 0.00, 0.00, 0.00);
      len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
      success = sendData(_sendRecvBuf, len);

      if (!success) {
        ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Attitude");
      }

      _lastMeasuredSentTime = now;

    }

    //send payload data
    if ((_lastMeasuredPayloadTime > ros::Time(0.0))
        && (_lastMeasuredPayloadTime - _lastMeasuredSentPayloadTime) > ros::Duration(0.1)) {
      mavlink_msg_uav_payload_data_pack(_systemID, 200, &_mavMessage, _payloadLon, _payloadLat, _altitudeMeasured,
          _depthMeasured, _payloadFlour);

      len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
      success = sendData(_sendRecvBuf, len);

      if (!success) {
        ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Payload Data");
      }

      _lastMeasuredSentPayloadTime = now;
    }

    _lastStatusTime = now;
  }

  if (_lookingForWaypoints > 0) {
    if ((now - _lookingForStartTime) > ros::Duration(5.0)) {
      ROS_WARN("No Waypoint data in 5 secs, dropping %d recv'd waypoints", _currentLookingForPoint);
      _lookingForWaypoints = -1;
    }
  }

  memset(_sendRecvBuf, 0, BUFFER_LENGTH);

  fd_set rfds;
  struct timeval tv;
  int retval;

  FD_ZERO(&rfds);
  FD_SET(_socket, &rfds);

  tv.tv_sec = 0;
  tv.tv_usec = 100000;

  retval = select((_socket + 1), &rfds, NULL, NULL, &tv);
  if (retval == -1) {
    perror("select()");
  } else if (retval) {
    /* FD_ISSET(0, &rfds) will be true. */
    struct sockaddr_in recvAddr;
    socklen_t fromlen = sizeof(recvAddr);
    int recsize = recvfrom(_socket, (void *) _sendRecvBuf, BUFFER_LENGTH, 0, (struct sockaddr *) &recvAddr, &fromlen);
    if (recsize > 0) {
      // Something received - print out all b\ytes and parse packet
      mavlink_message_t msg;
      mavlink_status_t status;

      for (int i = 0; i < recsize; ++i) {
        //uint8_t temp = _sendRecvBuf[i];
        if (mavlink_parse_char(MAVLINK_COMM_0, _sendRecvBuf[i], &msg, &status)) {
          // Packet received
          switch (msg.msgid) {
          case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t set_param;
            mavlink_msg_param_set_decode(&msg, &set_param);

            ROS_INFO("Got Set Param id %s value %f", set_param.param_id, set_param.param_value);
            std::map<std::string, double>::iterator pIter = _paramValues.find(set_param.param_id);
            if (pIter != _paramValues.end()) {
              if (string_contains(set_param.param_id, "dctrl_")) {
                _dcUpdateGainSrv.request.updateGainID = _paramDCIDs[pIter->first];
                _dcUpdateGainSrv.request.value = set_param.param_value;
                double temp = pIter->second;
                //Update the gain here to make QGC happy, DC will update it in the callback later as well
                pIter->second = set_param.param_value;
                sendParams();
                if (_dcUpdateGainClient.call(_dcUpdateGainSrv)) {
                  if (!_dcUpdateGainSrv.response.accepted) {
                    pIter->second = temp;
                    sendParams();
                    ROS_WARN("DC Update Service not Accepted!!  param ID = %s enum %d", pIter->first.c_str(),
                        (int) _paramDCIDs[pIter->first]);
                  }
                } else {
                  pIter->second = temp;
                  sendParams();
                  ROS_WARN("DC Update Service call Failed!!  param ID = %s enum %d", pIter->first.c_str(),
                      (int) _paramDCIDs[pIter->first]);
                }
              } else if (string_contains(set_param.param_id, "health_")) {
                _healthUpdateValSrv.request.updateValueID = _paramHealthIDs[pIter->first];
                _healthUpdateValSrv.request.value = set_param.param_value;
                double temp = pIter->second;
                // Update the value here to make QGC happy, will be updated in the callback later as well
                pIter->second = set_param.param_value;
                sendParams();
                if (_healthUpdateValClient.call(_healthUpdateValSrv)) {
                  if (!_healthUpdateValSrv.response.accepted) {
                    pIter->second = temp;
                    sendParams();
                    ROS_WARN("Health update service not accepted!!  param ID = %s enum %d", pIter->first.c_str(),
                        (int) _paramHealthIDs[pIter->first]);
                  }
                } else {
                  pIter->second = temp;
                  sendParams();
                  ROS_WARN("Health update service call failed!!  param ID = %s enum %d", pIter->first.c_str(),
                      (int) _paramHealthIDs[pIter->first]);
                }
              } else {
                ROS_WARN("Param ID %s not resolved to app", set_param.param_id);
              }
            } else {
              ROS_WARN("Unknown Param ID %s", set_param.param_id);
            }
          }
            break;
          case MAVLINK_MSG_ID_PARAM_VALUE: {
            mavlink_param_value_t param_var;
            mavlink_msg_param_value_decode(&msg, &param_var);

            ROS_INFO("Got parameter index %d of %d, name %s, value %f", param_var.param_index, param_var.param_count,
                param_var.param_id, param_var.param_value);
          }
            break;
          case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t mode;
            mavlink_msg_set_mode_decode(&msg, &mode);
            ROS_INFO("Got Mode command %d", mode.custom_mode);
          }
            break;
          case MAVLINK_MSG_ID_MISSION_ACK: {
            mavlink_mission_ack_t mavAck;
            mavlink_msg_mission_ack_decode(&msg, &mavAck);
            ROS_INFO("Got MISSION ack from mavlink target %d : %d -- type %d", mavAck.target_system,
                mavAck.target_component, mavAck.type);
          }
            break;
          case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t cmdAck;
            mavlink_msg_command_ack_decode(&msg, &cmdAck);
            ROS_INFO("Got COMMAND ack from mavlink -- command %d result %d", cmdAck.command, cmdAck.result);
            if (_waitingForCalibrationAck) {
              if (_calibrateAccel != ACCEL_CAL_OFF) {
                _calibrationStartTime = now;
                _sampleAccel = true;
              }

              snprintf(_statMsg, sizeof(_statMsg), "Sampling ACCEL -- Mode %d", _calibrateAccel);
              _statMsg[sizeof(_statMsg) - 1] = '\0';
              sendStatusText(_statMsg);
            }
          }
          case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            mavlink_mission_request_list_t requestList;
            mavlink_msg_mission_request_list_decode(&msg, &requestList);
            ROS_INFO("Waypoint request! -- sys %d, comp %d", requestList.target_system, requestList.target_component);
            if (requestList.target_system == _systemID) {
              //copy current list
              _cachedCurrent = _currentWaypoint;
              _cachedList.assign(_currentList.begin(), _currentList.end());
              mavlink_msg_mission_count_pack(_systemID, MAV_COMP_ID_MISSIONPLANNER, &_mavMessage, _gcsID,
                  MAV_COMP_ID_ALL, _cachedList.size());
              int len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
              success = sendData(_sendRecvBuf, len);
              if (!success) {
                ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Mission Count response");
              }
            }
          }
            break;
          case MAVLINK_MSG_ID_MISSION_REQUEST: {
            mavlink_mission_request_t missionRequest;
            mavlink_msg_mission_request_decode(&msg, &missionRequest);
            if (missionRequest.target_system == _systemID) {
              ROS_INFO("Mission request for seq %d", missionRequest.seq);
              if ((missionRequest.seq >= 0) && (missionRequest.seq < _cachedList.size())) {
                //TODO: should we use MISSIONPLANNER here?
                mavlink_msg_mission_item_pack(_systemID, MAV_COMP_ID_MISSIONPLANNER, &_mavMessage, _gcsID,
                    MAV_COMP_ID_ALL, missionRequest.seq, _cachedList[missionRequest.seq].coord_frame,
                    _cachedList[missionRequest.seq].command, ((_cachedCurrent == missionRequest.seq) ? 1 : 0),
                    (_cachedList[missionRequest.seq].auto_continue ? 1 : 0), _cachedList[missionRequest.seq].param1,
                    _cachedList[missionRequest.seq].param2, _cachedList[missionRequest.seq].param3,
                    _cachedList[missionRequest.seq].param4, _cachedList[missionRequest.seq].param5,
                    _cachedList[missionRequest.seq].param6, _cachedList[missionRequest.seq].param7);
                int len = mavlink_msg_to_send_buffer(_sendRecvBuf, &_mavMessage);
                success = sendData(_sendRecvBuf, len);
                if (!success) {
                  ROS_WARN_THROTTLE(30, "Bad Sendto for MavLink Mission Item response for waypoint seq %d",
                      missionRequest.seq);
                }
              }
            }
          }
            break;
          case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t commandOptions;
            mavlink_msg_command_long_decode(&msg, &commandOptions);
            ROS_INFO(
                "Command recv'd! -- sys %d, comp %d -- command %d, param1 %f, param2 %f, param3 %f, param4 %f, param5 %f, param6 %f, param7 %f",
                commandOptions.target_system, commandOptions.target_component, commandOptions.command,
                commandOptions.param1, commandOptions.param2, commandOptions.param3, commandOptions.param4,
                commandOptions.param5, commandOptions.param6, commandOptions.param7);
            if (commandOptions.target_system == _systemID) {
              switch (commandOptions.command) {
              case MAV_CMD_NAV_LAND:
                //stop mission
                _objCmdMsg.start_mission = false;
                _objCmdMsg.stop_mission = true;
                _objCmdMsg.pop_objective = false;
                _objCmdMsg.push_objective = false;
                _objCommandPub.publish(_objCmdMsg);
                break;
              case MAV_CMD_MISSION_START:
              case MAV_CMD_NAV_TAKEOFF:
              case MAV_CMD_OVERRIDE_GOTO:

                if (commandOptions.command == MAV_CMD_OVERRIDE_GOTO && commandOptions.param1 == MAV_GOTO_DO_HOLD) {
                  //stop mission
                  _objCmdMsg.start_mission = false;
                  _objCmdMsg.stop_mission = true;
                } else {
                  //start mission
                  _objCmdMsg.start_mission = true;
                  _objCmdMsg.stop_mission = false;
                }

                _objCmdMsg.pop_objective = false;
                _objCmdMsg.push_objective = false;
                _objCommandPub.publish(_objCmdMsg);
                break;
              case MAV_CMD_PREFLIGHT_CALIBRATION:
                ROS_INFO("Got Preflight Calibration!! Confirm %d -- param2(mag) %f, param3(tare) %f, param5(accel) %f",
                    commandOptions.confirmation, commandOptions.param2, commandOptions.param3, commandOptions.param5);
                if (commandOptions.param5 > 0.0) {
                  ROS_INFO("Accel Cal start!");
                  _calibrateAccel = ACCEL_CAL_FLAT;
                  _waitingForCalibrationAck = true;
                  _calibrationStartTime = now;
                  _sampleAccel = true;
                  _accelSamples.clear();
                  if (_verboseCalibration) {
                    ROS_INFO("Restarting accel calibration and resetting accelerometer samples");
                  }
                  snprintf(_statMsg, sizeof(_statMsg), "Place vehicle FLAT and level and press ACCEL");
                  _statMsg[sizeof(_statMsg) - 1] = '\0';
                  sendStatusText(_statMsg);
                }
                if (commandOptions.param2 > 0.0) {
                  ROS_INFO("Mag Cal start!");
                  _calibrateMag = MAG_CAL_HOLD;
                  _waitingForCalibrationAck = true;
                  _calibrationStartTime = now;
                  _sampleMag = false;
                  _magSamples.clear();
                  if (_verboseCalibration) {
                    ROS_INFO("Restarting mag calibration and resetting magnetometer samples");
                  }
                  snprintf(_statMsg, sizeof(_statMsg), "Mag Sampling starts in 5 secs...");
                  _statMsg[sizeof(_statMsg) - 1] = '\0';
                  sendStatusText(_statMsg);

                }
                break;
              }
            }
          }
            break;
          case MAVLINK_MSG_ID_MISSION_COUNT: {
            mavlink_mission_count_t wCount;
            mavlink_msg_mission_count_decode(&msg, &wCount);
            ROS_INFO("Got Mission Count %d", wCount.count);
            if (wCount.target_system == _systemID) {
              _lookingForWaypoints = wCount.count;
              _receivedList.clear();
              _currentLookingForPoint = 0;
              _lookingForStartTime = now;
              if (_lookingForWaypoints > 0) {
                sendWaypointRequest(_currentLookingForPoint);
              } else {
                sendMissionACK();
              }
            }
          }
            break;
          case MAVLINK_MSG_ID_MISSION_ITEM: {
            mavlink_mission_item_t missionItem;
            mavlink_msg_mission_item_decode(&msg, &missionItem);
            if (missionItem.target_system == _systemID) {
              ROS_INFO("Got Mission Item %d -- %d %d", missionItem.seq, _lookingForWaypoints, _currentLookingForPoint);

              if ((_lookingForWaypoints > 0) && (_currentLookingForPoint == missionItem.seq)) {

                ROS_INFO(
                    "Mission item number %d, looking for waypoint %d, currently looking for %d, received list size %d",
                    missionItem.seq, _lookingForWaypoints, _currentLookingForPoint, (int) _receivedList.size());

                //Fill in current list
                struct Waypoint waypoint;
                waypoint.coord_frame = missionItem.frame;
                waypoint.command = missionItem.command;
                waypoint.param1 = missionItem.param1;
                waypoint.param2 = missionItem.param2;
                waypoint.param3 = missionItem.param3;
                waypoint.param4 = missionItem.param4;
                waypoint.param5 = missionItem.x;
                waypoint.param6 = missionItem.y;
                waypoint.param7 = missionItem.z;
                waypoint.auto_continue = (missionItem.autocontinue > 0);
                _receivedList.push_back(waypoint);

                _recvPointMsg.param1.clear();
                _recvPointMsg.param2.clear();
                _recvPointMsg.param3.clear();
                _recvPointMsg.param4.clear();
                _recvPointMsg.param5.clear();
                _recvPointMsg.param6.clear();
                _recvPointMsg.param7.clear();
                _recvPointMsg.auto_continue.clear();

                ROS_INFO("Waypoint frame %d command %d 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f auto is %d",
                    missionItem.frame, missionItem.command, missionItem.param1, missionItem.param2, missionItem.param3,
                    missionItem.param4, missionItem.x, missionItem.y, missionItem.z, missionItem.autocontinue);
                if (_currentLookingForPoint == (_lookingForWaypoints - 1)) {
                  _recvPointMsg.coord_frame.clear();
                  _recvPointMsg.command.clear();
                  _recvPointMsg.param1.clear();
                  _recvPointMsg.param2.clear();
                  _recvPointMsg.param3.clear();
                  _recvPointMsg.param4.clear();
                  _recvPointMsg.param5.clear();
                  _recvPointMsg.param6.clear();
                  _recvPointMsg.param7.clear();
                  _recvPointMsg.auto_continue.clear();

                  sendMissionACK();
                  _recvPointMsg.number_of_points = _receivedList.size();
                  _recvPointMsg.current_point = 0;
                  for (vector<struct Waypoint>::iterator iter = _receivedList.begin(); iter != _receivedList.end();
                      ++iter) {
                    _recvPointMsg.coord_frame.push_back((*iter).coord_frame);
                    _recvPointMsg.command.push_back((*iter).command);
                    _recvPointMsg.param1.push_back((*iter).param1);
                    _recvPointMsg.param2.push_back((*iter).param2);
                    _recvPointMsg.param3.push_back((*iter).param3);
                    _recvPointMsg.param4.push_back((*iter).param4);
                    _recvPointMsg.param5.push_back((*iter).param5);
                    _recvPointMsg.param6.push_back((*iter).param6);
                    _recvPointMsg.param7.push_back((*iter).param7);
                    _recvPointMsg.auto_continue.push_back((*iter).auto_continue);
                  }
                  _recvPointListPub.publish(_recvPointMsg);
                  _lookingForWaypoints = -1;
                } else {
                  _currentLookingForPoint++;
                  sendWaypointRequest(_currentLookingForPoint);
                }
              }
            }
          }
            break;
          case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
            //Send clear list to objective control
            mavlink_mission_clear_all_t clearAll;
            mavlink_msg_mission_clear_all_decode(&msg, &clearAll);
            if (clearAll.target_system == _systemID) {
              _recvPointMsg.number_of_points = 0;
              _recvPointListPub.publish(_recvPointMsg);
              sendMissionACK();
            }
          }
            break;
          case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            ROS_INFO("Requesting param list");
            mavlink_param_request_list_t reqList;
            mavlink_msg_param_request_list_decode(&msg, &reqList);
            if (reqList.target_system == _systemID) {
              sendParams();
            }
          }
          case MAVLINK_MSG_ID_ADAPT_POLYGON: {
            mavlink_adapt_polygon_t polygon;
            mavlink_msg_adapt_polygon_decode(&msg, &polygon);
            //TODO: add target_system to message?
            //if( polygon.target_system == _systemID) {

            ROS_INFO("Received mavlink polygon message");

            _recvPolygonMsg.poly_lon.clear();
            _recvPolygonMsg.poly_lat.clear();

            //only supports quadilateral
            _recvPolygonMsg.number_of_points = 4;
            _recvPolygonMsg.poly_lon.push_back(polygon.poly_lon[0]);
            _recvPolygonMsg.poly_lon.push_back(polygon.poly_lon[1]);
            _recvPolygonMsg.poly_lon.push_back(polygon.poly_lon[2]);
            _recvPolygonMsg.poly_lon.push_back(polygon.poly_lon[3]);

            _recvPolygonMsg.poly_lat.push_back(polygon.poly_lat[0]);
            _recvPolygonMsg.poly_lat.push_back(polygon.poly_lat[1]);
            _recvPolygonMsg.poly_lat.push_back(polygon.poly_lat[2]);
            _recvPolygonMsg.poly_lat.push_back(polygon.poly_lat[3]);

            switch (polygon.poly_type) {
            case BOUNDING_POLYGON: {
              _recvPolygonMsg.polygon_mode = sandshark_msgs::MavlinkPolygon::BOUNDING_POLYGON;
              break;
            }
            case AVOID_POLYGON: {
              _recvPolygonMsg.polygon_mode = sandshark_msgs::MavlinkPolygon::BOUNDING_POLYGON;
              break;
            }
            case SURVEY_POLYGON: {
              _recvPolygonMsg.polygon_mode = sandshark_msgs::MavlinkPolygon::BOUNDING_POLYGON;
              break;
            }
            }
            _recvPolygonPub.publish(_recvPolygonMsg);

            //}
          }
          default:
            break;
          }
        }
      }
    } else if (recsize < 0) {
      fprintf( stderr, "Negative recsize, error = %d:%s\n", errno, strerror( errno));
    }
  } else {
    ROS_INFO("No data within 5 seconds");
  }

  memset(_sendRecvBuf, 0, BUFFER_LENGTH);

  return true;
}

void MavLinkDriver::handleSleep() {
  _rate->sleep();
}

uint64_t MavLinkDriver::microsSinceEpoch() {
  struct timeval tv;

  uint64_t micros = 0;

  gettimeofday(&tv, NULL);
  micros = ((uint64_t) tv.tv_sec) * 1000000 + tv.tv_usec;

  return micros;
}

void MavLinkDriver::handleCalibration() {
  ros::Time now = ros::Time::now();

  if (_calibrateAccel != ACCEL_CAL_OFF) {
    if (_sampleAccel) {
      switch (_calibrateAccel) {
      case ACCEL_CAL_FLAT:
        if (_verboseCalibration) {
          ROS_INFO("Need 100 samples to get to next step");
        }
        if (_accelSamples.size() > 100) {
          _calibrateAccel = ACCEL_CAL_LEFT;
          _waitingForCalibrationAck = true;
          _calibrationStartTime = now;
          _sampleAccel = false;
          snprintf(_statMsg, sizeof(_statMsg), "Place vehicle on its LEFT side and press ACCEL");
          _statMsg[sizeof(_statMsg) - 1] = '\0';
          sendStatusText(_statMsg);
        }
        break;
      case ACCEL_CAL_LEFT:
        if (_verboseCalibration) {
          ROS_INFO("Need 200 samples to get to next step");
        }
        if (_accelSamples.size() > 2 * 100) {
          _calibrateAccel = ACCEL_CAL_TOP;
          _waitingForCalibrationAck = true;
          _calibrationStartTime = now;
          _sampleAccel = false;
          snprintf(_statMsg, sizeof(_statMsg), "Place vehicle UPSIDE DOWN and ACCEL");
          _statMsg[sizeof(_statMsg) - 1] = '\0';
          sendStatusText(_statMsg);
        }
        break;
      case ACCEL_CAL_TOP:
        if (_verboseCalibration) {
          ROS_INFO("Need 300 samples to get to next step");
        }
        if (_accelSamples.size() > 3 * 100) {
          _calibrateAccel = ACCEL_CAL_RIGHT;
          _waitingForCalibrationAck = true;
          _calibrationStartTime = now;
          _sampleAccel = false;
          snprintf(_statMsg, sizeof(_statMsg), "Place vehicle on its RIGHT side and press ACCEL");
          _statMsg[sizeof(_statMsg) - 1] = '\0';
          sendStatusText(_statMsg);
        }
        break;
      case ACCEL_CAL_RIGHT:
        if (_verboseCalibration) {
          ROS_INFO("Need 400 samples to get to next step");
        }
        if (_accelSamples.size() > 4 * 100) {
          _calibrateAccel = ACCEL_CAL_NOSE;
          _waitingForCalibrationAck = true;
          _calibrationStartTime = now;
          _sampleAccel = false;
          snprintf(_statMsg, sizeof(_statMsg), "Place vehicle on its NOSE and press ACCEL");
          _statMsg[sizeof(_statMsg) - 1] = '\0';
          sendStatusText(_statMsg);
        }
        break;
      case ACCEL_CAL_NOSE:
        if (_verboseCalibration) {
          ROS_INFO("Need 500 samples to get to next step");
        }
        if (_accelSamples.size() > 5 * 100) {
          _calibrateAccel = ACCEL_CAL_TAIL;
          _waitingForCalibrationAck = true;
          _calibrationStartTime = now;
          _sampleAccel = false;
          snprintf(_statMsg, sizeof(_statMsg), "Place vehicle on its TAIL side and press ACCEL");
          _statMsg[sizeof(_statMsg) - 1] = '\0';
          sendStatusText(_statMsg);
        }
        break;
      case ACCEL_CAL_TAIL:
        if (_verboseCalibration) {
          ROS_INFO("Need 600 samples to get to next step");
        }
        if (_accelSamples.size() > 6 * 100) {
          _calibrateAccel = ACCEL_CAL_FLAT2;
          _waitingForCalibrationAck = true;
          _calibrationStartTime = now;
          _sampleAccel = false;
          snprintf(_statMsg, sizeof(_statMsg), "Place vehicle down on a LEVEL surface and press ACCEL");
          _statMsg[sizeof(_statMsg) - 1] = '\0';
          sendStatusText(_statMsg);
        }
        break;
      case ACCEL_CAL_FLAT2:
        if (_verboseCalibration) {
          ROS_INFO("Need 700 samples to get to next step");
        }
        if (_accelSamples.size() > 7 * 100) {
          _calibrateAccel = ACCEL_CAL_OFF;
          _sampleAccel = false;

          //Calibrate?

          ROS_INFO("\n\nReady to CALIBRATE have %d samples!\n", (int) _accelSamples.size());

          FILE *calFile = fopen("/data/app/accelcaldata.csv", "w");
          for (vector<Vector3d>::iterator iter = _accelSamples.begin(); iter != _accelSamples.end(); ++iter) {
            fprintf(calFile, "%f,%f,%f\n", (*iter).x(), (*iter).y(), (*iter).z());
          }
          fclose(calFile);

          Vector3d center, radii;
          if (alignedEllipsoidFit(_accelSamples, center, radii)) {
            snprintf(_statMsg, sizeof(_statMsg), "ACCEL Calibration Fit is good");
            ROS_INFO("ACCEL Cal Center is %f, %f, %f", center.x(), center.y(), center.z());
            ROS_INFO("ACCEL Cal Radii is %f, %f, %f", radii.x(), radii.y(), radii.z());

            _accelCalibSrv.request.accelOffset.clear();
            _accelCalibSrv.request.accelOffset.push_back(center.x());
            _accelCalibSrv.request.accelOffset.push_back(center.y());
            _accelCalibSrv.request.accelOffset.push_back(center.z());

            _accelCalibSrv.request.accelScaling.clear();
            _accelCalibSrv.request.accelScaling.push_back(radii.x());
            _accelCalibSrv.request.accelScaling.push_back(radii.y());
            _accelCalibSrv.request.accelScaling.push_back(radii.z());

            if (_accelCalibClient.call(_accelCalibSrv)) {
              if (_accelCalibSrv.response.accepted) {
                snprintf(_statMsg, sizeof(_statMsg), "ACCEL Cal Accepted!");
              } else {
                snprintf(_statMsg, sizeof(_statMsg), "ACCEL Cal REJECTED");
              }
            } else {
              snprintf(_statMsg, sizeof(_statMsg), "No Response - ACCEL CAL FAILED");
            }
          } else {
            snprintf(_statMsg, sizeof(_statMsg), "ACCEL Calibration Fit FAILED");
          }
          _statMsg[sizeof(_statMsg) - 1] = '\0';
          sendStatusText(_statMsg);
        }
        break;
      case ACCEL_CAL_OFF:
        //We shouldn't be here!
        break;
      }
    }
  } else if (_calibrateMag != MAG_CAL_OFF) {
    switch (_calibrateMag) {
    case MAG_CAL_HOLD: {
      ros::Duration elapsed = now - _calibrationStartTime;
      snprintf(_statMsg, sizeof(_statMsg), "Mag Cal will begin in %f seconds!", ceil(elapsed.toSec()));
      if (elapsed > ros::Duration(5.0)) {
        _sampleMag = true;
        _calibrateMag = MAG_CAL_SAMPLING;
      }
      _statMsg[sizeof(_statMsg) - 1] = '\0';
      sendStatusText(_statMsg);
    }
      break;
    case MAG_CAL_SAMPLING:
      if (_magSamples.size() > 1000) {
        _calibrateMag = MAG_CAL_FIT;
      }
      snprintf(_statMsg, sizeof(_statMsg), "Mag Cal needs %d samples!", (1000 - (int) _magSamples.size()));
      _statMsg[sizeof(_statMsg) - 1] = '\0';
      sendStatusText(_statMsg);
      break;
    case MAG_CAL_FIT:
      if (_magSamples.size() >= 1000) {
        _calibrateMag = MAG_CAL_OFF;
        _sampleMag = false;
        ROS_INFO("\n\nReady to CALIBRATE have %d samples!\n", (int) _magSamples.size());

        FILE *calFile = fopen("/data/app/magcaldata.csv", "w");
        for (vector<Vector3d>::iterator iter = _magSamples.begin(); iter != _magSamples.end(); ++iter) {
          fprintf(calFile, "%f,%f,%f\n", (*iter).x(), (*iter).y(), (*iter).z());
        }
        fclose(calFile);

        Matrix3d rotation;
        Vector3d center, radii;
        if (uniqueEllipsoidFit(_magSamples, center, radii, rotation)) {
          snprintf(_statMsg, sizeof(_statMsg), "MAG Calibration Fit is good");
          ROS_INFO("MAG Cal Center is %f, %f, %f", center.x(), center.y(), center.z());
          ROS_INFO("MAG Cal Radii is %f, %f, %f", radii.x(), radii.y(), radii.z());
          cout << "Mag Rotation: " << rotation << std::endl;

          _magCalibSrv.request.magOffset.clear();
          _magCalibSrv.request.magOffset.push_back(center.x());
          _magCalibSrv.request.magOffset.push_back(center.y());
          _magCalibSrv.request.magOffset.push_back(center.z());

          _magCalibSrv.request.magScaling.clear();
          _magCalibSrv.request.magScaling.push_back(radii.x());
          _magCalibSrv.request.magScaling.push_back(radii.y());
          _magCalibSrv.request.magScaling.push_back(radii.z());

          _magCalibSrv.request.magAdjust.clear();
          for (unsigned int row = 0; row < 3; ++row) {
            for (unsigned int col = 0; col < 3; ++col) {
              _magCalibSrv.request.magAdjust.push_back(rotation(row, col));
            }
          }
          if (_magCalibClient.call(_magCalibSrv)) {
            if (_magCalibSrv.response.accepted) {
              snprintf(_statMsg, sizeof(_statMsg), "MAG Cal Accepted!");
            } else {
              snprintf(_statMsg, sizeof(_statMsg), "MAG Cal REJECTED");
            }
          } else {
            snprintf(_statMsg, sizeof(_statMsg), "No Response - MAG CAL FAILED");
          }
        } else {
          snprintf(_statMsg, sizeof(_statMsg), "MAG Calibration Fit FAILED");
        }
        _statMsg[sizeof(_statMsg) - 1] = '\0';
        sendStatusText(_statMsg);
      }
      break;
    case MAG_CAL_OFF:
      //We shouldn't be here either
      break;
    }
  }
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::MavLinkDriver md;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) md, argc, argv);
}
