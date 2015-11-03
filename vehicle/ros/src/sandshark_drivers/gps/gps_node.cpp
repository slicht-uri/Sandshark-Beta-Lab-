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

#include <ros/ros.h>
#include <Eigen>
#include <sandshark_common/sqlite3.h>
#include <sandshark_common/proj_api.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_msgs/GPS.h>

using namespace std;
using namespace Eigen;
namespace bluefin {
namespace sandshark {

bool new_data;
vector<string> sqldata;
static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
  int i;
  for (i = 0; i < argc; i++) {
    if ((i + 1) > (int) sqldata.size()) {
      sqldata.push_back("NULL");
    }
    sqldata[i] = argv[i] ? argv[i] : "NULL";
  }
  new_data = true;
  return 0;
}

class GpsDriver: public TaskBase {
  private:
    bool _initialized;
    int _utm_zone;
    sqlite3 *_database;

    projPJ _utmDefinition;
    projPJ _latlngDefinition;

    ros::Publisher _gpsPub;
    sandshark_msgs::GPS _gpsMsg;

    double _lastEpoch;

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    GpsDriver() :
        TaskBase("GpsDriver", "gps") {
    }

    void handleSleep();
};

void GpsDriver::startupInitCallback() {
  ROS_INFO("Task init! - creating gps node\n");
  fflush( stdout);

  _gpsPub = _publicNode->advertise<sandshark_msgs::GPS>("gps", 10);

  fflush( stdout);
  _utm_zone = 19;
  _initialized = false;
  new_data = false;

  if (_logLocal && _logFile) {
    fprintf(_logFile,
        "ROSTime,\tEpoch,\tLatitude,\tLongitude,\tNorthings,\tEastings,\tUTMZone,\tAccuracy,\tSpeed,\tBearing\n");
  }

}

bool GpsDriver::doInitialize() {
  cleanup();

  char *zErrMsg = 0;

  int sqret = sqlite3_open("/data/data/com.bluefinrobotics.bluefingps/databases/BLUEFINGPSDB", &_database);

  if (sqret) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(_database));
    sqlite3_close(_database);
    return false;
  }

  sqret = sqlite3_exec(_database, "SELECT * FROM LOCATION_POINTS", callback, 0, &zErrMsg);
  if (sqret != SQLITE_OK) {
    fprintf( stderr, "SQL error: %s\n", zErrMsg);
    sqlite3_free(zErrMsg);
    return false;
  }

  _lastEpoch = 0.0;

  return true;
}

void GpsDriver::cleanup() {
  ROS_INFO("GPS Cleanup");
  fflush( stdout);
}

bool GpsDriver::doRun() {
  char *zErrMsg = 0;

  int sqret = sqlite3_exec(_database, "SELECT * FROM LOCATION_POINTS", callback, 0, &zErrMsg);
  if (sqret != SQLITE_OK) {
    fprintf( stderr, "SQL error: %s\n", zErrMsg);
    sqlite3_free(zErrMsg);
    return true;
  }

  bool haveLat = false;
  bool haveLng = false;
  double lat, lng;
  if (new_data && (sqldata.size() >= 10)) {
    if (sqldata[3] != "NULL") {
      lat = atof(sqldata[3].c_str());
      haveLat = true;
    }
    if (sqldata[4] != "NULL") {
      lng = atof(sqldata[4].c_str());
      haveLng = true;
    }

    fflush( stdout);
    if (!_initialized && haveLat && haveLng) {
      int zone = (int) ((lng + 180.0) / 6.0 + 1);
      bool south = (lat < 0);
      char utmInitString[256];
      snprintf(&utmInitString[0], sizeof(utmInitString), "+proj=utm +zone=%d +ellps=WGS84%s", zone,
          (south ? " +south" : ""));

      ROS_INFO("Lat %f Lng %f zone %d string %s", lat, lng, zone, utmInitString);
      fflush( stdout);

      _utmDefinition = pj_init_plus(utmInitString);
      if (!_utmDefinition) {
        ROS_ERROR("UTM Def failed!!");
        return false;
      }

      _latlngDefinition = pj_init_plus("+proj=latlong +ellps=WGS84");
      if (!_latlngDefinition) {
        ROS_ERROR("LatLng Def failed!!");
        return false;
      }

      _utm_zone = zone;
      _initialized = true;
    }

    new_data = false;

    if (_initialized && haveLat && haveLng) {
      double northings = lat * DEG_TO_RAD;
      double eastings = lng * DEG_TO_RAD;
      pj_transform(_latlngDefinition, _utmDefinition, 1, 1, &eastings, &northings, NULL);

      _gpsMsg.gmt_timestamp = sqldata[1];
      _gpsMsg.epoch = atof(sqldata[2].c_str());
      _gpsMsg.latitude = lat;
      _gpsMsg.longitude = lng;
      _gpsMsg.northings = northings;
      _gpsMsg.eastings = eastings;
      _gpsMsg.utm_zone = _utm_zone;
      _gpsMsg.altitude = atof(sqldata[5].c_str());
      _gpsMsg.accuracy = atof(sqldata[6].c_str());
      _gpsMsg.satellites_in_use = atoi(sqldata[7].c_str());
      _gpsMsg.speed = atof(sqldata[8].c_str());
      _gpsMsg.bearing = atof(sqldata[9].c_str());

      if (_gpsMsg.epoch > _lastEpoch && _gpsMsg.satellites_in_use > 5) {
        ROS_INFO("GPS Publishing! Lat %f Lng %f N %f E %f Epoch %f Last Epoch %f", lat, lng, northings, eastings,
            _gpsMsg.epoch, _lastEpoch);
        if (_logLocal && _logFile) {
          fprintf(_logFile, "%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%d,\t%f,\t%f,\t%f\n", ros::WallTime::now().toSec(),
              _gpsMsg.epoch, lat, lng, northings, eastings, _utm_zone, _gpsMsg.accuracy, _gpsMsg.speed,
              _gpsMsg.bearing);
          fflush(_logFile);
        }
        _gpsPub.publish(_gpsMsg);
        _lastEpoch = _gpsMsg.epoch;
      }
    }
  }
  return true;
}

void GpsDriver::handleSleep() {
  usleep(100000);
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::GpsDriver gd;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) gd, argc, argv);
}
