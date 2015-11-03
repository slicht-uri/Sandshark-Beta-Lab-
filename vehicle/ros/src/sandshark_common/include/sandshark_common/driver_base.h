#ifndef _BLUEFIN__SANDSHARK__SANDSHARK_COMMON__DRIVER_BASE_H_
#define _BLUEFIN__SANDSHARK__SANDSHARK_COMMON__DRIVER_BASE_H_

#include "task_base.h"

namespace bluefin {
namespace sandshark {
//DriverBase adds a device timeout mechanism to Task Base
class DriverBase: public TaskBase {
  private:

    int _deviceTimeouts;
    int _deviceTimeoutsLimit;
    int _deviceTimeoutsDivisor;
    int _deviceTimeoutsDivisorCounter;
  protected:

    void deviceTimeoutWarning(std::string wStr, ...);

    /**
     * Set up gpio pin
     * @param pin     gpio pin to set up
     * @param input   set up pin for input (true) or output (false)
     * @return        true if sucess, false if failure
     */
    bool gpioInit(int pin, bool input);

    /**
     * Release the gpio pin
     * @param pin     gpio pin to set up
     * @return        true if sucess, false if failure
     */
    bool gpioRelease(int pin);

    bool gpioWrite(int pin, uint8_t data);
    bool gpioReadOneChar(int pin, uint8_t & data);

    explicit DriverBase(std::string taskName, std::string topic_ns) :
        TaskBase(taskName, topic_ns), _deviceTimeouts(0), _deviceTimeoutsLimit(0), _deviceTimeoutsDivisor(0), _deviceTimeoutsDivisorCounter(
            0) {
    }

  public:
    virtual void startupInitialization();
    virtual void run();
    virtual ~DriverBase() {
    }
};
}
}

#endif //DRIVER_BASE_H
