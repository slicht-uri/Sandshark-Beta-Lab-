#include <android/sensor.h>
#include <sys/sysinfo.h>
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
#include <sys/socket.h>
#include <arpa/inet.h>

#include <Eigen>

#include <ros/ros.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

#include <sandshark_msgs/OrientationEuler.h>
#include <sandshark_msgs/OrientationQuaternion.h>
#include <sandshark_msgs/OrientationDebug.h>
#include <sandshark_msgs/MotionRaw.h>
#include <sandshark_msgs/MotionCorrected.h>
#include <sandshark_msgs/AccelCalib.h>
#include <sandshark_msgs/MagCalib.h>

#define LOOPER_ID_USER 3

using namespace std;
using namespace Eigen;
namespace bluefin {
namespace sandshark {

class MotionDriver: public DriverBase {
  private:
    ALooper* _looper;
    ASensorManager* _sensorManager;
    vector<const ASensor*> _sensorList;
    vector<ASensorEventQueue*> _queueList;
    vector<ASensorEvent> _eventList;

    ros::Publisher _orientPub;
    sandshark_msgs::OrientationEuler _orientMsg;

    ros::Publisher _orientDebugPub;
    sandshark_msgs::OrientationDebug _orientDebugMsg;

    ros::Publisher _orientQuaternionPub;
    sandshark_msgs::OrientationQuaternion _orientQuaternionMsg;

    ros::Publisher _rawPub;
    sandshark_msgs::MotionRaw _rawMsg;

    ros::Publisher _correctedPub;
    sandshark_msgs::MotionCorrected _correctedMsg;

    ros::ServiceServer _accelService;
    ros::ServiceServer _magService;

    struct timespec _start;
    bool _firstRun;
    bool _needSleep;
    Vector3d _correctedAccel;
    Vector3d _gravityAccel;
    Vector3d _linearAccel;

    Vector3d _accelOffset;
    Vector3d _accelScaling;

    Vector3d _correctedMag;

    Vector3d _magOffset;
    Vector3d _magScaling;
    Matrix3d _magAdjust;

    Quaternion<double> _attitudeEstimate;

    Quaternion<double> _rCQuaternion;

    double _alpha;
    double _beta;
    double _zeta;

    double _gyroBiasX;
    double _gyroBiasY;
    double _gyroBiasZ;

    double _firstBetaDuration;
    double _firstBeta;
    double _start_time;

    bool _imuFilterEnabled;
    bool _imuFilterAfterBeta;
    bool _imuFilterWithBadMag;
    double _minGoodMag;
    double _maxGoodMag;

    bool _sendUDP;
    std::string _sendIPAddress;
    int _sendIPPort;
    int _sendSocket;
    struct sockaddr_in _sendAddrStruct;
    bool _sendAndroidQuaternion;

    void convertAndPublishEuler();

    bool ahrsEnabled(double magMagnitude);
    void publishEulerAngles();
    void publishDebug();

    void reset(bool writeParams);

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    MotionDriver() :
        DriverBase("MotionDriver", "motion") {
    }

    bool handleAccelCalibUpdate(sandshark_msgs::AccelCalib::Request &req, sandshark_msgs::AccelCalib::Response &res);
    bool handleMagCalibUpdate(sandshark_msgs::MagCalib::Request &req, sandshark_msgs::MagCalib::Response &res);

    void handleSleep();
    void handleShutdown() {
      cleanup();
    }
};

void MotionDriver::startupInitCallback() {
  ROS_INFO("Task init! - creating orient node");
  fflush( stdout);

  _orientPub = _publicNode->advertise<sandshark_msgs::OrientationEuler>("orientation_euler", 1);
  _orientDebugPub = _publicNode->advertise<sandshark_msgs::OrientationDebug>("orientation_debug", 1);
  _orientQuaternionPub = _publicNode->advertise<sandshark_msgs::OrientationQuaternion>("orientation_quaternion", 1);
  _rawPub = _publicNode->advertise<sandshark_msgs::MotionRaw>("raw_sensors", 1);
  _correctedPub = _publicNode->advertise<sandshark_msgs::MotionCorrected>("corrected_sensors", 1);

  _accelService = _publicNode->advertiseService("calibrateAccel", &MotionDriver::handleAccelCalibUpdate, this);
  _magService = _publicNode->advertiseService("calibrateMag", &MotionDriver::handleMagCalibUpdate, this);

  _sensorManager = ASensorManager_getInstance();
  _looper = ALooper_forThread();
  if (_looper == NULL) {
    _looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ROS_INFO("Looper was NULL called prepare!");
  }
  ROS_INFO("Task init! Param stuff looper is %p", _looper);
  fflush( stdout);

  string key;
  vector<double> values;
  char comma;
  double value;

  _accelOffset << 0.0, 0.0, 0.0;
  values.clear();
  if (ros::param::search("accel_offset", key)) {
    string accel_offset;
    ros::param::get(key, accel_offset);

    istringstream iss(accel_offset);
    while (iss >> value) {
      values.push_back(value);
      iss >> comma;
    }

    if (values.size() == 3) {
      for (unsigned int i = 0; i < values.size(); ++i) {
        _accelOffset[i] = values[i];
      }
    }
  }
  cout << "Accel Offset: " << _accelOffset << std::endl;

  ROS_INFO("Task init! Param aOffset");
  fflush( stdout);

  _accelScaling << 9.8, 9.8, 9.8;
  values.clear();
  if (ros::param::search("accel_scaling", key)) {
    string accel_scaling;
    ros::param::get(key, accel_scaling);

    istringstream iss(accel_scaling);
    while (iss >> value) {
      values.push_back(value);
      iss >> comma;
    }

    if (values.size() == 3) {
      for (unsigned int i = 0; i < values.size(); ++i) {
        _accelScaling[i] = values[i];
      }
    }
  }

  cout << "Accel Scaling " << _accelScaling << std::endl;

  _magOffset << 0.0, 0.0, 0.0;
  values.clear();
  if (ros::param::search("mag_offset", key)) {
    string mag_offset;
    ros::param::get(key, mag_offset);

    istringstream iss(mag_offset);
    while (iss >> value) {
      values.push_back(value);
      iss >> comma;
    }

    if (values.size() == 3) {
      for (unsigned int i = 0; i < values.size(); ++i) {
        _magOffset[i] = (values[i]);
      }
    }
  }
  cout << "Mag Offset: " << _magOffset << std::endl;

  fflush( stdout);
  _magAdjust = Matrix3d::Identity();
  values.clear();
  if (ros::param::search("mag_adjust", key)) {
    string mag_adjust;
    ros::param::get(key, mag_adjust);

    istringstream iss(mag_adjust);
    while (iss >> value) {
      values.push_back(value);
      iss >> comma;
    }

    if (values.size() == 9) {
      for (unsigned int i = 0; i < values.size(); ++i) {
        int row = i / 3;
        int col = i % 3;
        ROS_INFO("i %d row %d col %d val %f", i, row, col, values[i]);
        fflush( stdout);
        _magAdjust(row, col) = values[i];
      }
    }
  }
  cout << "Mag Adjust: " << _magAdjust << std::endl;

  _magScaling << 1.0, 1.0, 1.0;
  values.clear();
  if (ros::param::search("mag_scaling", key)) {
    string mag_scaling;
    ros::param::get(key, mag_scaling);

    istringstream iss(mag_scaling);
    while (iss >> value) {
      values.push_back(value);
      iss >> comma;
    }

    if (values.size() == 3) {
      for (unsigned int i = 0; i < values.size(); ++i) {
        _magScaling[i] = values[i];
      }
    }
  }

  cout << "Mag Scaling: " << _magScaling << std::endl;

  ROS_INFO("Task init! Private Param stuff");
  fflush( stdout);
  _privateNode->param("alpha", _alpha, double(0.8));
  _privateNode->param("beta", _beta, double(0.2));
  _privateNode->param("zeta", _zeta, double(0.05));

  _privateNode->param("firstBetaDuration", _firstBetaDuration, double(10.0));
  _privateNode->param("firstBeta", _firstBeta, double(2.5));

  _privateNode->param("sendUDP", _sendUDP, false);
  _privateNode->param("sendIPAddress", _sendIPAddress, std::string("127.0.0.1"));
  _privateNode->param("sendIPPort", _sendIPPort, int(5555));
  ROS_INFO("SendIP %s : %d", _sendIPAddress.c_str(), _sendIPPort);
  _privateNode->param("sendAndroidQuaternion", _sendAndroidQuaternion, false);

  _privateNode->param("imuFilterEnabled", _imuFilterEnabled, false);
  _privateNode->param("imuFilterAfterBeta", _imuFilterAfterBeta, false);
  _privateNode->param("imuFilterWithBadMag", _imuFilterWithBadMag, false);
  _privateNode->param("minGoodMag", _minGoodMag, double(25)); //Units ?!?!?
  _privateNode->param("maxGoodMag", _maxGoodMag, double(65));

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
  ROS_INFO("Param alpha %f beta %f zeta %f", _alpha, _beta, _zeta);
  ROS_INFO("Task start! Param stuff complete");
  fflush( stdout);
}

bool MotionDriver::handleAccelCalibUpdate(sandshark_msgs::AccelCalib::Request &req,
    sandshark_msgs::AccelCalib::Response &res) {
  std::ostringstream strs;
  if ((req.accelOffset.size() < 3) || (req.accelScaling.size() < 3)) {
    res.accepted = false;
    return true;
  }

  _accelOffset << req.accelOffset[0], req.accelOffset[1], req.accelOffset[2];
  strs << _accelOffset[0] << "," << _accelOffset[1] << "," << _accelOffset[2];
  _privateNode->setParam("accel_offset", strs.str());

  strs.str("");
  strs.clear();

  _accelScaling << req.accelScaling[0], req.accelScaling[1], req.accelScaling[2];
  strs << _accelScaling[0] << "," << _accelScaling[1] << "," << _accelScaling[2];
  _privateNode->setParam("accel_scaling", strs.str());

  cout << "Service UPDATE Accel Offset: " << _accelOffset << std::endl;
  cout << "Service UPDATE Accel Scaling: " << _accelScaling << std::endl;
  res.accepted = true;
  reset(true);
  return true;
}

bool MotionDriver::handleMagCalibUpdate(sandshark_msgs::MagCalib::Request &req,
    sandshark_msgs::MagCalib::Response &res) {
  std::ostringstream strs;
  if ((req.magOffset.size() < 3) || (req.magScaling.size() < 3) || (req.magAdjust.size() < 9)) {
    res.accepted = false;
    return true;
  }

  _magOffset << req.magOffset[0], req.magOffset[1], req.magOffset[2];
  strs << _magOffset[0] << "," << _magOffset[1] << "," << _magOffset[2];
  _privateNode->setParam("mag_offset", strs.str());

  strs.str("");
  strs.clear();

  _magScaling << req.magScaling[0], req.magScaling[1], req.magScaling[2];
  strs << _magScaling[0] << "," << _magScaling[1] << "," << _magScaling[2];
  _privateNode->setParam("mag_scaling", strs.str());

  strs.str("");
  strs.clear();

  _magAdjust << req.magAdjust[0], req.magAdjust[1], req.magAdjust[2], req.magAdjust[3], req.magAdjust[4], req.magAdjust[5], req.magAdjust[6], req.magAdjust[7], req.magAdjust[8];
  strs << _magAdjust(0, 0) << "," << _magAdjust(0, 1) << "," << _magAdjust(0, 2) << "," << _magAdjust(1, 0) << ","
      << _magAdjust(1, 1) << "," << _magAdjust(1, 2) << "," << _magAdjust(2, 0) << "," << _magAdjust(2, 1) << ","
      << _magAdjust(2, 2);
  _privateNode->setParam("mag_adjust", strs.str());

  cout << "Service UPDATE Mag Offset: " << _magOffset << std::endl;
  cout << "Service UPDATE Mag Scaling: " << _magScaling << std::endl;
  cout << "Service UPDATE Mag Adjust: " << _magAdjust << std::endl;

  res.accepted = true;
  reset(true);
  return true;
}

bool MotionDriver::doInitialize() {
  cleanup();

  _needSleep = false;
  ASensorList senList;
  int numAllSensors = ASensorManager_getSensorList(_sensorManager, &senList);

  ROS_INFO("Task init! Motion numsen %d", numAllSensors);
  fflush( stdout);
  int rc;
  for (int i = 0; i < numAllSensors; ++i) {
    const ASensor *sensor = senList[i];
    ROS_INFO("Sensor Name %s Vendor %s Res %f Type %d", ASensor_getName(sensor), ASensor_getVendor(sensor),
        ASensor_getResolution(sensor), ASensor_getType(sensor));
    if ((std::string(ASensor_getName(sensor)) == "MPL magnetic field")
        || (std::string(ASensor_getName(sensor)) == "MPL accel") || (std::string(ASensor_getName(sensor)) == "MPL Gyro")
        || (_sendAndroidQuaternion && (std::string(ASensor_getName(sensor)) == "MPL rotation vector"))) {
      ASensorEventQueue* queue = ASensorManager_createEventQueue(_sensorManager, _looper, LOOPER_ID_USER, NULL, NULL);
      if (!queue) {
        ROS_INFO("error creating sensor event queue");
        return false;
      }
      rc = ASensorEventQueue_enableSensor(queue, sensor);
      if (rc < 0) {
        ROS_INFO("ASensorEventQueue_enableSensor error: %d", rc);
        return false;
      }
      rc = ASensorEventQueue_setEventRate(queue, sensor, ASensor_getMinDelay(sensor));
      if (rc < 0) {
        ROS_INFO("ASensorEventQueue_setEventRate error: %d", rc);
        return false;
      }

      fprintf( stderr, "Pushing back Sensor Name %s Vendor %s Res %f\n", ASensor_getName(sensor),
          ASensor_getVendor(sensor), ASensor_getResolution(sensor));
      _sensorList.push_back(sensor);
      _queueList.push_back(queue);
      _eventList.push_back(ASensorEvent());
    }
  }

  ROS_INFO("Task init! Motion sen complete %d", (int) _queueList.size());
  fflush( stdout);

  reset(false);

  ROS_INFO("Task init! Motion init complete event %d", (int) _eventList.size());
  fflush( stdout);

  return true;
}

void MotionDriver::reset(bool writeParams) {
  _firstRun = true;
  clock_gettime( CLOCK_REALTIME, &_start);
  _start_time = ros::WallTime::now().toSec();

  _gyroBiasX = 0.0;
  _gyroBiasY = 0.0;
  _gyroBiasZ = 0.0;
  _attitudeEstimate.w() = 1.0;
  _attitudeEstimate.x() = 0.0;
  _attitudeEstimate.y() = 0.0;
  _attitudeEstimate.z() = 0.0;

  if (writeParams) {
    ROS_INFO("Writing parameters...");
    //System calls are so dirty, there has to be a better way to do this
    system("rosparam dump /data/app/bluefin/opt/sandshark/share/sandshark_apps/config/motion.yaml /motion");
    ROS_INFO("Done");
  }
}

void MotionDriver::cleanup() {
  ROS_INFO("Task init! Motion cleanup");
  fflush( stdout);
  for (unsigned int i = 0; i < _sensorList.size(); i++) {
    ASensorEventQueue_disableSensor(_queueList[i], _sensorList[i]);
    ASensorManager_destroyEventQueue(_sensorManager, _queueList[i]);
  }

  _sensorList.clear();
  _queueList.clear();
  ROS_INFO("Task init! Cleanup done");
  fflush( stdout);
}

bool MotionDriver::ahrsEnabled(double magMagnitude) {
  //replace AHRS with IMU filter
  if (_imuFilterEnabled) {
    return false;
  }

  //Switch to IMU filter after first Beta
  if (_imuFilterAfterBeta) {
    double curTime = ros::WallTime::now().toSec();
    if ((curTime - _start_time) > _firstBetaDuration) {
      return false;
    }
  }

  if (_imuFilterWithBadMag) {
    if ((magMagnitude < _minGoodMag) || (magMagnitude > _maxGoodMag)) {
      return false;
    }
  }

  return true;
}

bool MotionDriver::doRun() {
  static struct timespec current;
  static double magX, magY, magZ, accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
  //A failure state of the magnetometer is that it will lock up and return
  //the same value forever, so we check for that and fail if it does
  static double lastMagX = 0, lastMagY = 0, lastMagZ = 0;
  static int magLockedCount = 0;
  static double o_x, o_y, o_z;
  static int magStat = 0, accelStat = 0, gyroStat = 0, count = 0;
  static Vector3d accelTemp, magTemp, accelNormalized, magNormalized;
  static bool haveMag = false, haveAccel = false, haveGyro = false, haveRotation = false;
  static Matrix<double, 3, 3> rotation;
  static Quaternion<double> attitudeConj;
  static Quaternion<double> magQuaternion;
  static Quaternion<double> gyroQuaternion, gyroError;
  static Quaternion<double> hQuaternion;
  static Quaternion<double> bQuaternion;
  static Matrix<double, 4, 1> stepVector;
  static Quaternion<double> stepQuaternion;
  static Matrix<double, 6, 4> J;
  static Matrix<double, 6, 1> F;
  static Matrix<double, 3, 4> imuJ;
  static Matrix<double, 3, 1> imuF;
  static double prev_elapsed = 0.0;
  static Vector4d rateOfChange;
  static char sendBuf[1400];
  static int noEventsCount = 0;

  clock_gettime( CLOCK_REALTIME, &current);
  //Assume start nsec is zero for simplicity
  double elapsed = (current.tv_sec - _start.tv_sec) + ((double) current.tv_nsec) / 1000000000;
  if (ASensorEventQueue_hasEvents(_queueList[0]) > 0) {  // if the master (first) sensor has new data
    for (unsigned int i = 0; i < _queueList.size(); ++i) {
      int ret = 0;
      int eventCount = 0;
      while ((ret = ASensorEventQueue_getEvents(_queueList[i], &_eventList[i], 1)) > 0) { //get all pending events
        //WARNING: do NOT put debug messages in here. The latency
        //will cause the motion driver to fail to get message
        //fast enough and will get stuck in an infinite loop

        clock_gettime( CLOCK_REALTIME, &current);
        double tempElapsed = (current.tv_sec - _start.tv_sec) + ((double) current.tv_nsec) / 1000000000.0;
        double lastEvent = (tempElapsed - elapsed);
        if (lastEvent > 2.0) { //No Sensor Data for 2 secs
          ROS_WARN("Getting current events for more than two seconds, driver failing!");
          return false;
        }

        eventCount += ret;
      }

      //make sure we got a new value
      if (ret < 0) {
        ROS_WARN("Error from android sensor manager (%d)", ret);
        setErrorMessage("Error from android sensor manager");
        noEventsCount = 0;
        //TODO: should we actually be failing here?
        return false;
      } else if (eventCount == 0) {
        noEventsCount++;
        if (noEventsCount > 5) {
          ROS_WARN("No events received from sensor manager!");
          setErrorMessage("Error from android sensor manager");
          noEventsCount = 0;
          //TODO: should we actually be failing here?
          return false;
        }
      } else {
        noEventsCount = 0;
        struct sysinfo info;
        int64_t uptime = 0;
        if (sysinfo(&info) == 0) {
          uptime = info.uptime;
        }

        float *d = _eventList[i].data;
        switch (_eventList[i].type) {
        case ASENSOR_TYPE_ACCELEROMETER:
          if ((_eventList[i].timestamp / 1000000000) - uptime > 2) {
            ROS_WARN("Processing old accelerometer data from sensor!!");
          }
          accelX = d[0];
          accelY = d[1];
          accelZ = d[2];
          accelStat = _eventList[i].acceleration.status;
          accelTemp(0) = accelX;
          accelTemp(1) = accelY;
          accelTemp(2) = accelZ;
          if (_firstRun) {
            _gravityAccel = accelTemp;
          }
          haveAccel = true;
          accelTemp -= _accelOffset;
          _correctedAccel = accelTemp;
          accelTemp(0) = _correctedAccel(0) / _accelScaling(0);
          accelTemp(1) = _correctedAccel(1) / _accelScaling(1);
          accelTemp(2) = _correctedAccel(2) / _accelScaling(2);
          _correctedAccel = accelTemp * 9.81;
          break;
        case ASENSOR_TYPE_MAGNETIC_FIELD:
          if ((_eventList[i].timestamp / 1000000000) - uptime > 2) {
            ROS_WARN("Processing old magnetometer data from sensor!!");
          }
          magX = d[0];
          magY = d[1];
          magZ = d[2];
          if (magX == lastMagX && magY == lastMagY && magZ == lastMagZ) {
            magLockedCount++;
            if (magLockedCount > 10) {
              //Something bad is happening! mag has locked up
              //Restart so we can get it unlocked
              ROS_WARN("Magnetometer has locked up!");
              setErrorMessage("Magnetometer has locked up!");
              return false;
            }
          } else {
            magLockedCount = 0;
          }
          lastMagX = magX;
          lastMagY = magY;
          lastMagZ = magZ;

          magStat = _eventList[i].magnetic.status;
          haveMag = true;
          magTemp(0) = magX;
          magTemp(1) = magY;
          magTemp(2) = magZ;
          magTemp -= _magOffset;
          //_correctedMag = magTemp;
          _correctedMag = magTemp.transpose() * _magAdjust;
          magTemp(0) = _correctedMag(0) / _magScaling(0);
          magTemp(1) = _correctedMag(1) / _magScaling(1);
          magTemp(2) = _correctedMag(2) / _magScaling(2);
          _correctedMag = magTemp.transpose() * _magAdjust.transpose();
          break;
        case ASENSOR_TYPE_GYROSCOPE:
          if ((_eventList[i].timestamp / 1000000000) - uptime > 2) {
            ROS_WARN("Processing old gyroscope data from sensor!!");
          }
          gyroX = d[0];
          gyroY = d[1];
          gyroZ = d[2];
          gyroStat = _eventList[i].vector.status;
          haveGyro = true;
          break;
        case 11:
          o_x = d[0];
          o_y = d[1];
          o_z = d[2];
          //                    o_w = d[3];
          haveRotation = true;
          break;
        }
      }
    }

    if (haveMag && haveAccel && haveGyro) {

      _rawMsg.accel_x = accelX;
      _rawMsg.accel_y = accelY;
      _rawMsg.accel_z = accelZ;
      _rawMsg.accel_status = accelStat;

      _rawMsg.mag_x = magX;
      _rawMsg.mag_y = magY;
      _rawMsg.mag_z = magZ;
      _rawMsg.mag_status = magStat;

      _rawMsg.gyro_x = gyroX;
      _rawMsg.gyro_y = gyroY;
      _rawMsg.gyro_z = gyroZ;
      _rawMsg.gyro_status = gyroStat;

      _rawPub.publish(_rawMsg);

      _gravityAccel = _alpha * _gravityAccel + (1 - _alpha) * _correctedAccel;
      _linearAccel = _correctedAccel - _gravityAccel;

      _correctedMsg.timestamp = current.tv_sec + ((double) current.tv_nsec) / 1000000000.0;
      _correctedMsg.accel_x = _correctedAccel(0);
      _correctedMsg.accel_y = _correctedAccel(1);
      _correctedMsg.accel_z = _correctedAccel(2);
      _correctedMsg.linear_x = _linearAccel(0);
      _correctedMsg.linear_y = _linearAccel(1);
      _correctedMsg.linear_z = _linearAccel(2);
      _correctedMsg.gravity_x = _gravityAccel(0);
      _correctedMsg.gravity_y = _gravityAccel(1);
      _correctedMsg.gravity_z = _gravityAccel(2);
      _correctedMsg.accel_status = accelStat;

      _correctedMsg.mag_x = _correctedMag(0);
      _correctedMsg.mag_y = _correctedMag(1);
      _correctedMsg.mag_z = _correctedMag(2);
      _correctedMsg.mag_status = magStat;

      _correctedMsg.gyro_x = gyroX;
      _correctedMsg.gyro_y = gyroY;
      _correctedMsg.gyro_z = gyroZ;
      _correctedMsg.gyro_status = gyroStat;

      _correctedPub.publish(_correctedMsg);

      accelNormalized = _gravityAccel / _gravityAccel.norm();
      magNormalized = _correctedMag / _correctedMag.norm();

      if (ahrsEnabled(_correctedMag.norm())) {
        magQuaternion.w() = 0.0;
        magQuaternion.x() = magNormalized.x();
        magQuaternion.y() = magNormalized.y();
        magQuaternion.z() = magNormalized.z();

        hQuaternion = _attitudeEstimate * (magQuaternion * _attitudeEstimate.conjugate());
        bQuaternion.w() = 0.0;
        bQuaternion.x() = sqrt(hQuaternion.x() * hQuaternion.x() + hQuaternion.y() * hQuaternion.y());
        bQuaternion.y() = 0.0;
        bQuaternion.z() = hQuaternion.z();

        F[0] = 2.0 * (_attitudeEstimate.x() * _attitudeEstimate.z() - _attitudeEstimate.w() * _attitudeEstimate.y())
            - accelNormalized.x();
        F[1] = 2.0 * (_attitudeEstimate.w() * _attitudeEstimate.x() + _attitudeEstimate.y() * _attitudeEstimate.z())
            - accelNormalized.y();
        F[2] = 2.0
            * (0.5 - _attitudeEstimate.x() * _attitudeEstimate.x() - _attitudeEstimate.y() * _attitudeEstimate.y())
            - accelNormalized.z();
        F[3] = 2.0 * bQuaternion.x()
            * (0.5 - _attitudeEstimate.y() * _attitudeEstimate.y() - _attitudeEstimate.z() * _attitudeEstimate.z())
            + 2.0 * bQuaternion.z()
                * (_attitudeEstimate.x() * _attitudeEstimate.z() - _attitudeEstimate.w() * _attitudeEstimate.y())
            - magNormalized.x();
        F[4] = 2.0 * bQuaternion.x()
            * (_attitudeEstimate.x() * _attitudeEstimate.y() - _attitudeEstimate.w() * _attitudeEstimate.z())
            + 2.0 * bQuaternion.z()
                * (_attitudeEstimate.w() * _attitudeEstimate.x() + _attitudeEstimate.y() * _attitudeEstimate.z())
            - magNormalized.y();
        F[5] = 2.0 * bQuaternion.x()
            * (_attitudeEstimate.w() * _attitudeEstimate.y() + _attitudeEstimate.x() * _attitudeEstimate.z())
            + 2.0 * bQuaternion.z()
                * (0.5 - _attitudeEstimate.x() * _attitudeEstimate.x() - _attitudeEstimate.y() * _attitudeEstimate.y())
            - magNormalized.z();

        J(0, 0) = -2.0 * _attitudeEstimate.y();
        J(0, 1) = 2.0 * _attitudeEstimate.z();
        J(0, 2) = -2.0 * _attitudeEstimate.w();
        J(0, 3) = 2.0 * _attitudeEstimate.x();
        J(1, 0) = 2.0 * _attitudeEstimate.x();
        J(1, 1) = 2.0 * _attitudeEstimate.w();
        J(1, 2) = 2.0 * _attitudeEstimate.z();
        J(1, 3) = 2.0 * _attitudeEstimate.y();
        J(2, 0) = 0.0;
        J(2, 1) = -4.0 * _attitudeEstimate.x();
        J(2, 2) = -4.0 * _attitudeEstimate.y();
        J(2, 3) = 0.0;
        J(3, 0) = -2.0 * bQuaternion.z() * _attitudeEstimate.y();
        J(3, 1) = 2.0 * bQuaternion.z() * _attitudeEstimate.z();
        J(3, 2) = -4.0 * bQuaternion.x() * _attitudeEstimate.y() - 2.0 * bQuaternion.z() * _attitudeEstimate.w();
        J(3, 3) = -4.0 * bQuaternion.x() * _attitudeEstimate.z() + 2.0 * bQuaternion.z() * _attitudeEstimate.x();
        J(4, 0) = -2.0 * bQuaternion.x() * _attitudeEstimate.z() + 2.0 * bQuaternion.z() * _attitudeEstimate.x();
        J(4, 1) = 2.0 * bQuaternion.x() * _attitudeEstimate.y() + 2.0 * bQuaternion.z() * _attitudeEstimate.w();
        J(4, 2) = 2.0 * bQuaternion.x() * _attitudeEstimate.x() + 2.0 * bQuaternion.z() * _attitudeEstimate.z();
        J(4, 3) = -2.0 * bQuaternion.x() * _attitudeEstimate.w() + 2.0 * bQuaternion.z() * _attitudeEstimate.y();
        J(5, 0) = 2.0 * bQuaternion.x() * _attitudeEstimate.y();
        J(5, 1) = 2.0 * bQuaternion.x() * _attitudeEstimate.z() - 4.0 * bQuaternion.z() * _attitudeEstimate.x();
        J(5, 2) = 2.0 * bQuaternion.x() * _attitudeEstimate.w() - 4.0 * bQuaternion.z() * _attitudeEstimate.y();
        J(5, 3) = 2.0 * bQuaternion.x() * _attitudeEstimate.x();

        stepVector = (J.transpose() * F);
        stepVector.normalize();
      } else {
        imuF[0] = 2.0 * (_attitudeEstimate.x() * _attitudeEstimate.z() - _attitudeEstimate.w() * _attitudeEstimate.y())
            - accelNormalized.x();
        imuF[1] = 2.0 * (_attitudeEstimate.w() * _attitudeEstimate.x() + _attitudeEstimate.y() * _attitudeEstimate.z())
            - accelNormalized.y();
        imuF[2] = 2.0
            * (0.5 - _attitudeEstimate.x() * _attitudeEstimate.x() - _attitudeEstimate.y() * _attitudeEstimate.y())
            - accelNormalized.z();

        imuJ(0, 0) = -2.0 * _attitudeEstimate.y();
        imuJ(0, 1) = 2.0 * _attitudeEstimate.z();
        imuJ(0, 2) = -2.0 * _attitudeEstimate.w();
        imuJ(0, 3) = 2.0 * _attitudeEstimate.x();
        imuJ(1, 0) = 2.0 * _attitudeEstimate.x();
        imuJ(1, 1) = 2.0 * _attitudeEstimate.w();
        imuJ(1, 2) = 2.0 * _attitudeEstimate.z();
        imuJ(1, 3) = 2.0 * _attitudeEstimate.y();
        imuJ(2, 0) = 0.0;
        imuJ(2, 1) = -4.0 * _attitudeEstimate.x();
        imuJ(2, 2) = -4.0 * _attitudeEstimate.y();
        imuJ(2, 3) = 0.0;

        stepVector = (imuJ.transpose() * imuF);
        stepVector.normalize();
      }
      stepQuaternion.w() = stepVector[0];
      stepQuaternion.x() = stepVector[1];
      stepQuaternion.y() = stepVector[2];
      stepQuaternion.z() = stepVector[3];
      gyroError = (_attitudeEstimate.conjugate() * stepQuaternion);

      double timestep = (elapsed - prev_elapsed);
      if (timestep < 0.0 || timestep > 2.0) {
        ROS_INFO("\ntime %f\n", timestep);
      }
      double curZeta = _zeta;
      double curTime = ros::WallTime::now().toSec();
      if ((curTime - _start_time) < _firstBetaDuration) {
        curZeta = 0.0;
      }

      _gyroBiasX += 2.0 * gyroError.x() * timestep * curZeta;
      _gyroBiasY += 2.0 * gyroError.y() * timestep * curZeta;
      _gyroBiasZ += 2.0 * gyroError.z() * timestep * curZeta;
      gyroQuaternion.w() = 0.0;
      gyroQuaternion.x() = gyroX - _gyroBiasX;
      gyroQuaternion.y() = gyroY - _gyroBiasY;
      gyroQuaternion.z() = gyroZ - _gyroBiasZ;

      _rCQuaternion = (_attitudeEstimate * gyroQuaternion);
      rateOfChange[0] = _rCQuaternion.w();
      rateOfChange[1] = _rCQuaternion.x();
      rateOfChange[2] = _rCQuaternion.y();
      rateOfChange[3] = _rCQuaternion.z();

      rateOfChange *= 0.5;
      if ((curTime - _start_time) < _firstBetaDuration) {
        rateOfChange -= _firstBeta * stepVector.transpose();
      } else {
        rateOfChange -= _beta * stepVector.transpose();
      }

      _attitudeEstimate.w() += rateOfChange[0] * timestep;
      _attitudeEstimate.x() += rateOfChange[1] * timestep;
      _attitudeEstimate.y() += rateOfChange[2] * timestep;
      _attitudeEstimate.z() += rateOfChange[3] * timestep;
      _attitudeEstimate.normalize();

      _orientQuaternionMsg.w = _attitudeEstimate.w();
      _orientQuaternionMsg.x = _attitudeEstimate.x();
      _orientQuaternionMsg.y = _attitudeEstimate.y();
      _orientQuaternionMsg.z = _attitudeEstimate.z();

      publishEulerAngles();
      publishDebug();
      _orientQuaternionPub.publish(_orientQuaternionMsg);

      if (_sendUDP && (count % 10 == 0)) {
        if (_sendAndroidQuaternion) {
          if (haveRotation) {
            snprintf(&sendBuf[0], sizeof(sendBuf), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                ros::WallTime::now().toSec(), accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ,
                _attitudeEstimate.w(), _attitudeEstimate.x(), _attitudeEstimate.y(), _attitudeEstimate.z(),
                _orientMsg.yaw, _orientMsg.roll, _orientMsg.pitch,
                sqrt(1.0 - ((o_x * o_x) + (o_y * o_y) + (o_z * o_z))), o_x, o_y, o_z);

          }
        } else {
          snprintf(&sendBuf[0], sizeof(sendBuf), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
              ros::WallTime::now().toSec(),
              //correctedAccel(0), correctedAccel(1), correctedAccel(2),
              accelX, accelY, accelZ, gyroX, gyroY, gyroZ,
              //correctedMag(0), correctedMag(1), correctedMag(2),
              magX, magY, magZ, _attitudeEstimate.w(), _attitudeEstimate.x(), _attitudeEstimate.y(),
              _attitudeEstimate.z(),
              //SEq_1, SEq_2, SEq_3, SEq_4,
              _orientMsg.yaw, _orientMsg.roll, _orientMsg.pitch);
        }
        sendBuf[sizeof(sendBuf) - 1] = '\0';
        int ret = sendto(_sendSocket, &sendBuf[0], strlen(sendBuf), 0, (struct sockaddr *) &_sendAddrStruct,
            sizeof(struct sockaddr));
        if (ret < 0) {
          fprintf( stderr, "SendTo failed ret %d - %d:%s\n", ret, errno, strerror( errno));
        }
      }
      prev_elapsed = elapsed;
      count++;
      haveMag = false;
      haveAccel = false;
      haveGyro = false;
    }
  } else {
    double lastEvent = (elapsed - prev_elapsed);
    if (lastEvent > 2.0) { //No Sensor Data for 2 secs

      elapsed = 0.0;
      prev_elapsed = 0.0;
      return false;
    }
    _needSleep = true;
  }

  _firstRun = false;
  return true;
}

void MotionDriver::handleSleep() {
  if (_needSleep) {
    usleep(10000);
  }
  _needSleep = false;
}

void MotionDriver::publishDebug() {
  //publish gyro bias and the rCQuaternion

  _orientDebugMsg.gyro_bias_x = _gyroBiasX;
  _orientDebugMsg.gyro_bias_y = _gyroBiasY;
  _orientDebugMsg.gyro_bias_z = _gyroBiasZ;

  _orientDebugMsg.rc_x = _rCQuaternion.x();
  _orientDebugMsg.rc_y = _rCQuaternion.y();
  _orientDebugMsg.rc_z = _rCQuaternion.z();
  _orientDebugMsg.rc_w = _rCQuaternion.w();

  _orientDebugPub.publish(_orientDebugMsg);
}

void MotionDriver::publishEulerAngles() {

  double froll = 0.0;
  double fpitch = 0.0;
  double fazimuth = 0.0;

  Matrix3d qrotation = _attitudeEstimate.matrix();
  //Rotate into vehicle orientation
  Matrix3d rotation = qrotation * AngleAxisd(0.5 * M_PI, Vector3d::UnitY());

  //rotation matrix to euler, pitch and azimuth are inverted
  froll = atan2(rotation(2, 1), rotation(2, 2));
  fpitch = asin(rotation(2, 0));
  fazimuth = atan2(rotation(1, 0), rotation(0, 0));

  froll *= 180.0 / M_PI;
  fpitch *= 180.0 / M_PI;
  fazimuth *= 180.0 / M_PI;
  if (fazimuth < 0.0) {
    fazimuth += 360.0;
  }

  if (fazimuth > 360.0) {
    fazimuth -= 360.0;
  }

  _orientMsg.roll = froll;
  _orientMsg.yaw = fazimuth;
  _orientMsg.pitch = fpitch;

  _orientPub.publish(_orientMsg);

}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::MotionDriver md;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) md, argc, argv);
}
