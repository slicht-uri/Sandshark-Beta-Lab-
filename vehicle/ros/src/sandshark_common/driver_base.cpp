#include <sandshark_common/driver_base.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace bluefin {
 namespace sandshark {
 
  void DriverBase::startupInitialization() {
      TaskBase::startupInitialization();
      printf("DriverBase startupInitialization\n");
      _deviceTimeouts = 0;
      _privateNode->param( "deviceTimeoutsLimit", _deviceTimeoutsLimit, 10 );
      _privateNode->param( "deviceTimeoutsDivisor", _deviceTimeoutsDivisor, 5 );
      if(_deviceTimeoutsDivisor < 2) {
          ROS_WARN("deviceTimeoutsDivisor set to %d. "
              "deviceTimeoutsLimit will never be reached.", _deviceTimeoutsDivisor);
      }
      _deviceTimeoutsDivisorCounter = 0;
  }
 
  void DriverBase::run() {
      //if enough runs have passed, subtract a devicetimeout
      if(_deviceTimeoutsDivisorCounter >= _deviceTimeoutsDivisor){
          _deviceTimeoutsDivisorCounter = 0;
          //only decrement to 0
          if (_deviceTimeouts > 0) {
              _deviceTimeouts--;
          }
      }
      _deviceTimeoutsDivisorCounter++;
 
       //if there have been too many device timeouts in a small number of runs, 
      //put the driver in an error state with an appropriate error string
      if(_deviceTimeouts >= _deviceTimeoutsLimit) {
          //set the error state and message
          _state = TB_ERROR;
          setErrorMessage( "Device Timeout Limit Exceeded" );
          //reset the number of timeouts
          _deviceTimeouts = 0;
      }
 
      //Do the normal TaskBase run
      TaskBase::run();
  }
 
  void DriverBase::deviceTimeoutWarning(std::string wStr, ...) {
      char buff[wStr.size() * 2 + 500]; //should be big enough
      _deviceTimeouts++;
 
      const char* fmt = wStr.c_str();
      va_list args;
      va_start(args, wStr);
      vsprintf(buff, fmt, args);
      va_end(args);
      ROS_WARN("%s", buff);
  }
 
  bool DriverBase::gpioInit(int pin, bool input) {
      if( pin < 0 ) {
        ROS_INFO( "GPIO Init pin is negative, init skipped." );
        return true;
      }
      
      //Write _pin to /sys/class/gpio/export to have sys create
      //a GPIO interface for our use
      char buf[40];
      sprintf( buf, "%d", pin );
      int setupFd = open( "/sys/class/gpio/export", O_WRONLY );
      if( setupFd != -1 ) {
          ssize_t ret = write( setupFd, buf, strlen( buf ) );
          close( setupFd );
 
          //check to see if the write was successful
          if( ret == -1) {
              ROS_WARN( "GPIO export failed, errno %d:%s", errno, strerror( errno ) );
              return false;
          }
      } else {
          ROS_WARN( "GPIO export file open failed, errno %d:%s", errno, strerror( errno ) );
          return false;
      }
 
      //Write to the direction file to make our GPIO an input or output
      sprintf( buf, "/sys/class/gpio/gpio%d/direction", pin );
      setupFd = open( buf, O_WRONLY );
      if( setupFd != -1 ) {
          ssize_t ret = write( setupFd, 
                          ( input ? "in" : "out" ), 
                         ( input ? 2 : 3 ) );
          close( setupFd );
         
          //check to see if the write was successful
          if( ret == -1) {
              ROS_WARN( "GPIO direction failed, errno %d:%s", errno, strerror( errno ) );
              return false;
          }
      } else {
          ROS_WARN( "GPIO direction file open failed, errno %d:%s", errno, strerror( errno ) );
          return false;
      }
 
      return true;
  }
 
  bool DriverBase::gpioRelease(int pin) {
      if( pin < 0 ) {
        return true;
      }

      //Unexport the gpio to free it up
      int setupFd = open( "/sys/class/gpio/unexport", O_WRONLY );
      if( setupFd != -1 ) {
          char buf[40];
          sprintf( buf, "%d", pin );
          ssize_t ret = write( setupFd, buf, strlen( buf ) );
          close( setupFd );
  
          if( ret == -1) {
              ROS_WARN( "GPIO release failed, errno %d:%s", errno, strerror( errno ) );
              return false;
          }
      } else {
          ROS_WARN( "GPIO release file open failed, errno %d:%s", errno, strerror( errno ) );
          return false;
      }
  
      return true;
  }
 
  bool DriverBase::gpioWrite( int pin, uint8_t data ) {
      if( pin < 0 ) {
        return true;
      }

      static char buf[64];    
      //Write to the direction file to make our GPIO an input or output
      sprintf( buf, "/sys/class/gpio/gpio%d/value", pin );
      int setupFd = open( buf, O_WRONLY );
      if( setupFd != -1 ) {
          ssize_t ret = write( setupFd, &data, 1 );
          close( setupFd );
          
          //check to see if the write was successful
          if( ret == -1) {
              ROS_WARN( "GPIO write failed, errno %d:%s", errno, strerror( errno ) );
              return false;
          }
      } else {
          ROS_WARN( "GPIO write file open failed, errno %d:%s", errno, strerror( errno ) );
          return false;
      }
 
      return true;
  }
 
  bool DriverBase::gpioReadOneChar( int pin, uint8_t & data ) {
      if( pin < 0 ) {
        data = 0;
        return true;
      }

      static char buf[64];    
      //Write to the direction file to make our GPIO an input or output
      sprintf( buf, "/sys/class/gpio/gpio%d/value", pin );
      int setupFd = open( buf, O_RDONLY );
      if( setupFd != -1 ) {
          ssize_t ret = read( setupFd, &data, 1 );
          close( setupFd );
         
          //check to see if the write was successful
          if( ret == -1) {
              ROS_WARN( "GPIO read failed, errno %d:%s", errno, strerror( errno ) );
              return false;
          }
      } else {
          ROS_WARN( "GPIO read file open failed, errno %d:%s", errno, strerror( errno ) );
          return false;
      }
 
      return true;
  }
 
 }
}

