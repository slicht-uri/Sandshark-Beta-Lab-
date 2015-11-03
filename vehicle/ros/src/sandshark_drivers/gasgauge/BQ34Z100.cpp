#include <stdint.h>
#include "BQ34Z100.h"

BQ34Z100::BQ34Z100() {
}

bool BQ34Z100::initialize(const char* filename) {
  _open = true;
  if ((_fd = open(filename, O_RDWR)) < 0) {
    //Something went wrong, exit 1
    printf("\nFailed to open the i2c bus\n");
    _open = false;
    return false;
  }

  //ioctl
  if (ioctl(_fd, I2C_SLAVE, BQ34Z100_ADDRESS) < 0) {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    return false;
  }

  return true;
}

BQ34Z100::~BQ34Z100() {
  close(_fd);
}

bool BQ34Z100::isOpen() {
  return _open;
}

bool BQ34Z100::_read_value(int command, int* value) {

  for (int i = 0; i < 2; i++) {
    _txbuf[0] = command + i;

    if (write(_fd, _txbuf, 1) != 1) {
      return false;
    } else {
      if (read(_fd, &_rxbuf[i], 1) != 1) {
        return false;
      }
    }
  }

  *value = (_rxbuf[0] + (_rxbuf[1] << 8));
  return true;
}

bool BQ34Z100::read_SOC(float* SOC) {
  int value;
  if (!_read_value(BQ34Z100_SOC, &value)) {
    return false;
  } else {
    uint16_t v = (uint16_t) value;
    *SOC = (float) v;
    return true;
  }
}

bool BQ34Z100::read_RM(float* RM) {
  int value;
  if (!_read_value(BQ34Z100_RM, &value)) {
    return false;
  } else {
    uint16_t v = (uint16_t) value;
    //Remaining Capacity is in mAh, divide by 1k to get Ah.
    *RM = (float) v / 1000;
    return true;
  }
}

bool BQ34Z100::read_FCC(float* FCC) {
  int value;
  if (!_read_value(BQ34Z100_FCC, &value)) {
    return false;
  } else {
    uint16_t v = (uint16_t) value;
    //Full Charge Capacity is in mAh, divide by 1k to get Ah.
    *FCC = (float) v / 1000;
    return true;
  }
}

bool BQ34Z100::read_VOLT(float* VOLT) {
  int value;
  if (!_read_value(BQ34Z100_VOLT, &value)) {
    return false;
  } else {
    uint16_t v = (uint16_t) value;
    //Voltage is in mV, divide by 1k to get V.
    *VOLT = (float) v / 1000;
    return true;
  }
}

bool BQ34Z100::read_AI(float* AI) {
  int value;
  if (!_read_value(BQ34Z100_AI, &value)) {
    return false;
  } else {
    int16_t v = (int16_t) value;
    //Average Current is in mA, divide by 1k to get A.
    *AI = (float) v;
    return true;
  }
}

bool BQ34Z100::read_TEMP(float* TEMP) {
  int value;
  if (!_read_value(BQ34Z100_TEMP, &value)) {
    return false;
  } else {
    uint16_t v = (uint16_t) value;
    //Temperature is in 1/10 degree Kelvin.
    *TEMP = (float) (v / 10 - 272.15);
    return true;
  }
}

bool BQ34Z100::Read(float* SOC, float* RM, float* FCC, float* VOLT, float* AI, float* TEMP) {

  bool soc = read_SOC(*&SOC);
  bool rm = read_RM(*&RM);
  bool fcc = read_FCC(*&FCC);
  bool volt = read_VOLT(*&VOLT);
  bool ai = read_AI(*&AI);
  bool temp = read_TEMP(*&TEMP);

  if (soc && rm && fcc && volt && ai && temp) {
    return true;
  } else {
    return false;
  }
}
