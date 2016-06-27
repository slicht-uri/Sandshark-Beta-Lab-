#include "MS89BSD.h"

#include <ros/ros.h>
#include <sandshark_msgs/Depth.h>
#include <sandshark_msgs/Environmental.h>
#include <sandshark_common/main.h>
#include <sandshark_common/task_base.h>
#include <sandshark_common/driver_base.h>

using namespace std;
namespace bluefin {
namespace sandshark {

class EnvDepthDriver: public DriverBase {
  private:
    MS89BSD *_depthI2C;
    
    int _i2cfd;

    ros::Publisher _depthPub;
    sandshark_msgs::Depth _depthMsg;
    ros::Publisher _envPub;
    sandshark_msgs::Environmental _envMsg;

    string _baseDepthName;
    string _baseEnvName;
    bool _haveDepth;
    
    string _i2cPort;
    int _enableGPIOPin;

    ros::Rate * _rate;

    float convertATMtoDBar(float & pressure_in_atm);

    void cleanup();
  protected:
    void startupInitCallback();
    bool doInitialize();
    bool doRun();
  public:
    EnvDepthDriver() :
        DriverBase("EnvDepthDriver", "envdepth"), _depthI2C( NULL) {
    }

    void handleSleep();
    virtual ~EnvDepthDriver() {
    }
};

void EnvDepthDriver::startupInitCallback() {
  _privateNode->param("haveDepth", _haveDepth, false );

  if( _haveDepth ) {
      _privateNode->param("baseDepthName", _baseDepthName, std::string( "depth" ) );
      _depthPub = _publicNode->advertise<sandshark_msgs::Depth>( std::string("/") + _baseDepthName + "/depth", 1);
  }

  _privateNode->param("baseEnvName", _baseEnvName, std::string( "environmental" ) );
  _envPub = _publicNode->advertise<sandshark_msgs::Environmental>( std::string("/") + _baseEnvName + "/environmental", 1);
  
  _privateNode->param("i2cPort", _i2cPort, std::string("/dev/i2c-4"));
  _privateNode->param("enableGPIOPin", _enableGPIOPin, (int) -1);

  if (!gpioInit(_enableGPIOPin, false)) {
    ROS_ERROR("GPIO Init Failed! Continuing since it may already exist");
  }

  _rate = new ros::Rate(10);
}

void EnvDepthDriver::cleanup() {
  //Shut off the buffers
  gpioWrite(_enableGPIOPin, '0');

  if( _depthI2C ) {
    delete _depthI2C;
    _depthI2C = 0;
  }
}

bool EnvDepthDriver::doInitialize() {
  cleanup();

  //Enable the I2C buffers
  if (!gpioWrite(_enableGPIOPin, '1')) {
    ROS_ERROR("Failed to enable LED I2C Buffers!");
    return false;
  }

  if( ( _i2cfd = open( _i2cPort.c_str(), O_RDWR) ) < 0 ) {
    ROS_ERROR( "Failed to open the i2c bus at %s.  Errno %d: %s", _i2cPort.c_str(), errno, strerror( errno ) );
    return false;
  }
  
  if( _haveDepth ) {
    _depthI2C = new MS89BSD( 0x77 );
    if( !_depthI2C->init( _i2cfd ) ) {
      ROS_ERROR( "Failed to init Depth Sensor!" );
      return false;
    }
  }
  
  return true;
}

bool EnvDepthDriver::doRun() {
  if( _haveDepth ) {
    float pressure, temperature;

    if( !_depthI2C->Read( _i2cfd, &pressure, &temperature ) ) {
      return false;
    }

    _depthMsg.pressure = convertATMtoDBar(pressure);
    _depthMsg.temperature = temperature;
    _depthPub.publish(_depthMsg);
  }

  return true;
}

void EnvDepthDriver::handleSleep() {
  _rate->sleep();
}

float EnvDepthDriver::convertATMtoDBar(float & pressure_in_atm) {
  return pressure_in_atm * (101325.0f / 10000.0f);
}

}
}

int main(int argc, char *argv[]) {
  bluefin::sandshark::EnvDepthDriver edd;
  return bluefin::sandshark::app_main((bluefin::sandshark::TaskBase&) edd, argc, argv);
}
