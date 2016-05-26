#include <Eigen>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/proj_api.h>

#include <ros/ros.h>
#include <sandshark_msgs/OrientationQuaternion.h>
#include <sandshark_msgs/OrientationEuler.h>
#include <sandshark_msgs/MotionCorrected.h>
#include <sandshark_msgs/GPS.h>
#include <sandshark_msgs/Depth.h>
#include <sandshark_msgs/Altitude.h>
#include <sandshark_msgs/Speed.h>
#include <sandshark_msgs/Navigation.h>
#include <sandshark_msgs/Velocities.h>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;
using namespace Eigen;
namespace bluefin {
namespace sandshark {

class NavigationApp: public TaskBase {
  private:
    ros::Subscriber _orientEulerSub;
    ros::Subscriber _gpsSub;
    ros::Subscriber _depthSub;
    ros::Subscriber _altitudeSub;
    ros::Subscriber _dcSpeedSub;

    double _gpsNorthings, _gpsEastings, _gpsBearing, _gps_speed, _gpsEpoch;
    string _gpsGMTTimestamp;
    int _gpsUTMZone;
    bool _gpsUpdated;
    bool _gpsIsSouth;
    double _prevNorthings, _prevEastings;
    double _prevDepth, _prevAltitude;

    bool _prevValid;

    double _curYaw, _curRoll, _curPitch;
    double _prevYaw, _prevRoll, _prevPitch;
    bool _orientValid;

    double _tarePressure;
    bool _haveTare;
    double _depthPressure;
    ros::Time _lastDepthTime;

    double _altitude;
    ros::Time _lastAltTime;

    double _estimatedSpeed;
    ros::Time _lastSpeedTime;

    bool _dcSpeedValid;
    ros::Time _lastDeadReck;

    double _accelSpeed;
    bool _csValid;

    ros::Publisher _navPub;
    sandshark_msgs::Navigation _navMsg;

    ros::Publisher _velocitiesPub;
    sandshark_msgs::Velocities _velocitiesMsg;

    int _deadreckTest;
    double _ignoreGpsForXSeconds;
    double _prevDeadreckTestGPSEpoch;
    double _surfaceDepth;
    double _acceptableTranslationalWackness;
    double _acceptableRotationalWackness;
    double _maxAccelSpeed;

    bool _shouldCheckRots;

    bool _sendUDP;
    std::string _sendIPAddress;
    int _sendIPPort;
    int _sendSocket;
    struct sockaddr_in _sendAddrStruct;

    bool _initialized;
    int _utm_zone;
    projPJ _utmDefinition;
    projPJ _latlngDefinition;

    double calculateDepth(double latitude, double pressure);

    ros::Rate * _rate;

    void invalidateNavMsg(std::string s);

  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
    void cleanup();
  public:
    NavigationApp() :
        TaskBase("Navigation", "navigation") {
    }

    void msgOrientEulerCallback(const sandshark_msgs::OrientationEuler::ConstPtr & msg);
    void msgGpsCallback(const sandshark_msgs::GPS::ConstPtr & msg);
    void msgDepthCallback(const sandshark_msgs::Depth::ConstPtr & msg);
    void msgAltitudeCallback(const sandshark_msgs::Altitude::ConstPtr & msg);
    void msgDCSpeedCallback(const sandshark_msgs::Speed::ConstPtr & msg);

    bool isWackness(std::string &wackness_reason);

    void handleSleep();
};

TaskBase *getTaskBase() {
  return new NavigationApp();
}

void NavigationApp::msgOrientEulerCallback(const sandshark_msgs::OrientationEuler::ConstPtr & msg) {
  _curYaw = msg->yaw;
  _curRoll = msg->roll;
  _curPitch = msg->pitch;
  _orientValid = true;
}

void NavigationApp::msgGpsCallback(const sandshark_msgs::GPS::ConstPtr & msg) {
  _gpsEastings = msg->eastings;
  _gpsNorthings = msg->northings;
  _gpsBearing = msg->bearing;
  _gps_speed = msg->speed;
  _gpsEpoch = msg->epoch;
  _gpsGMTTimestamp = msg->gmt_timestamp;
  _gpsUTMZone = msg->utm_zone;
  _gpsIsSouth = (msg->latitude < 0.0);
  _gpsUpdated = true;
}

void NavigationApp::msgDepthCallback(const sandshark_msgs::Depth::ConstPtr & msg) {
  _depthPressure = msg->pressure;
  _lastDepthTime = ros::Time::now();
}

void NavigationApp::msgAltitudeCallback(const sandshark_msgs::Altitude::ConstPtr & msg) {
  _altitude = msg->altitude;
  _lastAltTime = ros::Time::now();
}

void NavigationApp::msgDCSpeedCallback(const sandshark_msgs::Speed::ConstPtr & msg) {
  _estimatedSpeed = msg->speed_estimate;
  _lastSpeedTime = ros::Time::now();
  _dcSpeedValid = true;
}

void NavigationApp::startupInitCallback() {
  _orientEulerSub = _publicNode->subscribe("/motion/orientation_euler", 1, &NavigationApp::msgOrientEulerCallback,
      this);
  _gpsSub = _publicNode->subscribe("/gps/gps", 1, &NavigationApp::msgGpsCallback, this);
  _depthSub = _publicNode->subscribe("/depth/depth", 1, &NavigationApp::msgDepthCallback, this);
  _altitudeSub = _publicNode->subscribe("/altitude/altitude", 1, &NavigationApp::msgAltitudeCallback, this);
  _dcSpeedSub = _publicNode->subscribe("/dynamiccontrol/estimated_speed", 1, &NavigationApp::msgDCSpeedCallback, this);

  _navPub = _publicNode->advertise<sandshark_msgs::Navigation>("navState", 10);
  _velocitiesPub = _publicNode->advertise<sandshark_msgs::Velocities>("velocities", 10);

  _privateNode->param("deadrecktest", _deadreckTest, int(0));
  _privateNode->param("ignoreGpsForXSeconds", _ignoreGpsForXSeconds, double(120.0));
  _privateNode->param("surfaceDepth", _surfaceDepth, double(0.25));
  _privateNode->param("acceptableTranslationalWackness", _acceptableTranslationalWackness, double(1.0));
  _privateNode->param("acceptableRotationalWackness", _acceptableRotationalWackness, double(7.5));
  _prevDeadreckTestGPSEpoch = 0.0;

  _accelSpeed = 0.0;

  _shouldCheckRots = false;

  if (_logLocal && _logFile) {
    fprintf(_logFile,
        "ROSTime,\tGPSEpoch,\tNavEpoch,\tNavNorthings,\tNavEastings,\tGPSNorthings,\tGPSEastings,\tUTMZone,\tSpeed,\tBearing,\tPitch,\tRoll\n");
  }

  _privateNode->param("sendUDP", _sendUDP, false);
  _privateNode->param("sendIPAddress", _sendIPAddress, std::string("127.0.0.1"));
  _privateNode->param("sendIPPort", _sendIPPort, int(5556));
  ROS_INFO("SendIP %s : %d", _sendIPAddress.c_str(), _sendIPPort);

  _sendSocket = -1;
  if (_sendUDP) {
    _sendSocket = socket( AF_INET, SOCK_DGRAM, 0);
    if (_sendSocket < 0) {
      fprintf( stderr, "Socket is bad!\n");
      exit(-1);
    }

    struct sockaddr_in myaddr;
    memset((char *) &myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl( INADDR_ANY);
    myaddr.sin_port = htons(0);
    if (::bind(_sendSocket, (struct sockaddr *) &myaddr, sizeof(myaddr)) < 0) {
      fprintf( stderr, "bind failed! %d:%s\n", errno, strerror( errno));
      exit(-1);
    }

    memset(&_sendAddrStruct, 0, sizeof(_sendAddrStruct));
    _sendAddrStruct.sin_addr.s_addr = inet_addr(_sendIPAddress.c_str());
    _sendAddrStruct.sin_family = AF_INET;
    _sendAddrStruct.sin_port = htons(_sendIPPort);

  }
  _rate = new ros::Rate(20);
}

bool NavigationApp::doInitialize() {
  _gpsUpdated = false;
  _prevValid = false;
  _orientValid = false;
  _dcSpeedValid = false;

  _initialized = false;

  _prevAltitude = 0.0;
  _prevDepth = 0.0;
  return true;
}

void NavigationApp::cleanup() {
  //nothing to do...
}

bool NavigationApp::doRun() {
  static char sendBuf[1400];
  _navMsg.is_valid = true;
  _navMsg.invalid_reason = "";

  //ROS_INFO( "Nav::doRun Enter %f", ros::WallTime::now().toSec() );

  if (_deadreckTest > 0) {
    if (_gpsUpdated) {
      double elapsed = (_gpsEpoch - _prevDeadreckTestGPSEpoch) / 1000.0;
      if (elapsed >= _ignoreGpsForXSeconds) {
        _prevDeadreckTestGPSEpoch = _gpsEpoch;
      } else {
        _gpsUpdated = false;
      }
    }
  }

  if (_gpsUpdated) {
    _prevNorthings = _gpsNorthings;
    _prevEastings = _gpsEastings;
    _prevValid = true;
    _gpsUpdated = false;
    _navMsg.gmt_timestamp = _gpsGMTTimestamp;
    _navMsg.epoch = _gpsEpoch;
    _navMsg.quality = 1;
  } else {
    _navMsg.quality = 0;
  }

  if (_orientValid && _prevValid) {

    ros::Time now = ros::Time::now();
    ros::Duration deadReckDelta = now - _lastDeadReck;
    double deltaTime = deadReckDelta.toSec();

    _navMsg.computed_time = now;

    // Don't dead reckon if the delta is more than 0.4 seconds
    //TODO: may want to revisit how this is done
    if (deltaTime > 0.4) {
      deltaTime = 0.0;
    }

    //the speed estimate from dc already takes pitch into account
    double velX = 0.0;
    double velY = 0.0;

    if (_initialized && (now - _lastSpeedTime) < ros::Duration(5.0)) {
      velX = _estimatedSpeed * sin(((-1.0 * _curYaw) / 180.0) * M_PI);
      velY = _estimatedSpeed * cos(((-1.0 * _curYaw) / 180.0) * M_PI);

      _lastDeadReck = now;
    } else {
      //no more speed estimate, abort
      invalidateNavMsg("Nav stopped receiving speed estimate");
    }

    //only check for xy wackness if the vehicle is below surface depth, as it is perfectly resonable
    //to jump a large distance when receiving a first gps hit
    if (_navMsg.depth > _surfaceDepth) {
      double dx = velX * deltaTime;
      double dy = velY * deltaTime;
      if (dx > _acceptableTranslationalWackness || dx < -1 * _acceptableTranslationalWackness) {
        invalidateNavMsg("Preposterous change in eastings detected");
      }
      if (dy > _acceptableTranslationalWackness || dy < -1 * _acceptableTranslationalWackness) {
        invalidateNavMsg("Preposterous change in northings detected");
      }
    }

    _velocitiesMsg.east_velocity = velX;
    _velocitiesMsg.north_velocity = velY;

    _prevEastings += velX * deltaTime;
    _prevNorthings += velY * deltaTime;
    _navMsg.northings = _prevNorthings;
    _navMsg.eastings = _prevEastings;
    _navMsg.utm_zone = _gpsUTMZone;
    _navMsg.depth = 0.0;
    _navMsg.altitude = 0.0;
    _navMsg.gps_speed = _gps_speed;
    _navMsg.speed = _estimatedSpeed;

    //make sure no obscene changes in orientation occurred - no excuse for that, even on surface
    //only check after the first time
    if (_shouldCheckRots) {
      //yaw we need to account for wrapping at 360
      double wrappedYaw = 0.0;
      // if we have jumped too far, make nav invalid
      if (_curYaw > _prevYaw + _acceptableRotationalWackness
          || _curYaw < _prevYaw - _acceptableRotationalWackness) {
        //unless _currYaw and _prevYaw are close to 360 and/or 0
        if((0.0 + _acceptableRotationalWackness > _curYaw &&
            _curYaw > 0.0 - _acceptableRotationalWackness) &&
            (360.0 + _acceptableRotationalWackness > _prevYaw &&
             _prevYaw > 360.0 - _acceptableRotationalWackness)) {
        } else if((0.0 + _acceptableRotationalWackness > _prevYaw &&
            _prevYaw > 0.0 - _acceptableRotationalWackness) &&
            (360.0 + _acceptableRotationalWackness > _curYaw &&
             _curYaw > 360.0 - _acceptableRotationalWackness)) {
        } else {
          invalidateNavMsg("Preposterous change in Yaw detected");
        }
      }
      //no worries for pitch and roll wrapping.  If they wrap, there are
      //bigger issues to worry about
      if (_curPitch > _prevPitch + _acceptableRotationalWackness
          || _curPitch < _prevPitch - _acceptableRotationalWackness) {
        invalidateNavMsg("Preposterous change in Pitch detected");
      }
      if (_curRoll > _prevRoll + _acceptableRotationalWackness
          || _curRoll < _prevRoll - _acceptableRotationalWackness) {
        invalidateNavMsg("Preposterous change in Roll detected");
      }
    } else {
      _shouldCheckRots = true;
    }

    _prevYaw = _curYaw;
    _prevPitch = _curPitch;
    _prevRoll = _curRoll;

    _navMsg.bearing = -1.0 * _curYaw;
    if (_navMsg.bearing < 0.0) {
      _navMsg.bearing += 360.0;
    }
    if (_navMsg.bearing > 360.0) {
      _navMsg.bearing -= 360.0;
    }
    _navMsg.pitch = _curPitch;
    _navMsg.roll = _curRoll;

    if (!_initialized) {
      char utmInitString[256];
      snprintf(&utmInitString[0], sizeof(utmInitString), "+proj=utm +zone=%d +ellps=WGS84%s", _gpsUTMZone,
          (_gpsIsSouth ? " +south" : ""));

      _utmDefinition = pj_init_plus(utmInitString);
      if (!_utmDefinition) {
        ROS_ERROR("UTM Def failed!!\n");
        return false;
      }

      _latlngDefinition = pj_init_plus("+proj=latlong +ellps=WGS84");
      if (!_latlngDefinition) {
        ROS_ERROR("LatLng Def failed!!\n");
        return false;
      }

      _utm_zone = _gpsUTMZone;
      _initialized = true;
    }

    double lat = 0.0, lng = 0.0;
    bool latlng_valid = false;
    if (_initialized) {
      lat = _prevNorthings;
      lng = _prevEastings;
      int pret = pj_transform(_utmDefinition, _latlngDefinition, 1, 1, &lng, &lat, NULL);
      if (pret == 0) {
        latlng_valid = true;
      } else {
        latlng_valid = false;
        //ROS_ERROR("pj_transform returned invalid lat/lng - %d:%s", pret, pj_strerrno(pret));
      }
      lat *= RAD_TO_DEG;
      lng *= RAD_TO_DEG;
    }

    if (latlng_valid) {
      _navMsg.latitude = lat;
      _navMsg.longitude = lng;
    }

    now = ros::Time::now();

    if (_initialized && latlng_valid && (now - _lastDepthTime) < ros::Duration(0.5)) {
      _navMsg.depth = calculateDepth(lat, _depthPressure);
      //make sure we haven't had a ludicrous change in depth
      if (_navMsg.depth > _prevDepth + _acceptableTranslationalWackness
          || _navMsg.depth < _prevDepth - _acceptableTranslationalWackness) {
        invalidateNavMsg("Impossible change in depth detected");
      }
      _velocitiesMsg.down_velocity = (_navMsg.depth - _prevDepth) / deltaTime;
      _prevDepth = _navMsg.depth;
    } else if ((now - _lastDepthTime) >= ros::Duration(1.0)) {
      invalidateNavMsg("Nav stopped receiving depth data");
    }

    if (_logLocal && _logFile) {
      fprintf(_logFile, "%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%d,\t%f,\t%f,\t%f,\t%f,\t%f\n",
          ros::WallTime::now().toSec(), _gpsEpoch, _navMsg.epoch, _prevNorthings, _prevEastings, _gpsNorthings,
          _gpsEastings, _gpsBearing, _gpsUTMZone, _navMsg.speed, _curYaw, _navMsg.bearing, _navMsg.pitch, _navMsg.roll);
      fflush(_logFile);
    }

    if (_sendUDP) {
      snprintf(&sendBuf[0], sizeof(sendBuf), "%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%d,\t%f,\t%f,\t%f,\t%f,\t%f\n",
          ros::WallTime::now().toSec(), _gpsEpoch, _navMsg.epoch, _prevNorthings, _prevEastings, _gpsNorthings,
          _gpsEastings, _gpsBearing, _gpsUTMZone, _navMsg.speed, _curYaw, _navMsg.bearing, _navMsg.pitch, _navMsg.roll);

      sendBuf[sizeof(sendBuf) - 1] = '\0';
      int ret = sendto(_sendSocket, &sendBuf[0], strlen(sendBuf), 0, (struct sockaddr *) &_sendAddrStruct,
          sizeof(struct sockaddr));
      if (ret < 0) {
        fprintf( stderr, "SendTo failed ret %d - %d:%s\n", ret, errno, strerror( errno));
      }
    }

    if (_initialized && (now - _lastAltTime) < ros::Duration(10.0)) {
      _navMsg.altitude = _altitude;
      //altitude is somewhat filtered so if this happens its a huge red flag
      if (_altitude > _prevAltitude + _acceptableTranslationalWackness
          || _altitude < _prevAltitude - _acceptableTranslationalWackness) {
        invalidateNavMsg("Impossible change in altitude detected");
      }
      _prevAltitude = _altitude;
    } else {
      _navMsg.altitude = 0.0;
      _prevAltitude = 0.0;
      //if we haven't received altimiter data for 10 seconds, abort
      invalidateNavMsg("Nav stopped receiving altimiter data");
    }

    _navPub.publish(_navMsg);
    _orientValid = false;

    //TODO - need to do velocities if not dead reckoning
    _velocitiesPub.publish(_velocitiesMsg);
  }

  ROS_INFO( "Nav doRun exiting" );
  return true;
}

double NavigationApp::calculateDepth(double latitude, double pressure) {
  double depth_in_meters = 0.0;

  double lat_rad = latitude * DEG_TO_RAD;
  double pres_dbar = pressure; //Need conversion here!?

  if (!_haveTare) {
    _tarePressure = pres_dbar;
    _haveTare = true;
  }
  // Calculate depth in meters from latitude and pressure From
  // SBE Application Note 69
  double st2 = sin(lat_rad);
  st2 *= st2;
  double dbar = pres_dbar - _tarePressure;
  double C = (((-1.82e-15 * dbar + 2.279e-10) * dbar - 2.2512e-05) * dbar + 9.72659) * dbar;

  double gr = 9.780318 * ((2.3600e-05 * st2 + 5.2788e-3) * st2 + 1) + 1.092e-06 * dbar;
  depth_in_meters = C / gr; // meters
  //ROS_ERROR("PRESSURE IS %f DEPTH IS %f",pressure,depth_in_meters);

  return depth_in_meters;
}

void NavigationApp::handleSleep() {
  //ROS_INFO( "Nav::handleSleep enter" );
  _rate->sleep();
  //usleep(20000);
  //ROS_INFO( "Nav::handleSleep exiting" );
}

void NavigationApp::invalidateNavMsg(std::string s) {
  _navMsg.is_valid = false;
  _navMsg.invalid_reason = s;
  ROS_ERROR("Nav Validation False: %s\n", s.c_str());
}
}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::NavigationApp as;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) as, argc, argv);
}
