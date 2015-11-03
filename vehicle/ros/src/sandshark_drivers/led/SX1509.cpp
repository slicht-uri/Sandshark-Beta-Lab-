#include <stdint.h>
#include "SX1509.h"

uint8_t _reg_i_on[16] = { REG_I_ON_0, REG_I_ON_1, REG_I_ON_2, REG_I_ON_3, REG_I_ON_4, REG_I_ON_5, REG_I_ON_6,
REG_I_ON_7, REG_I_ON_8, REG_I_ON_9, REG_I_ON_10, REG_I_ON_11, REG_I_ON_12, REG_I_ON_13, REG_I_ON_14, REG_I_ON_15 };

uint8_t _reg_t_on[16] = { REG_T_ON_0, REG_T_ON_1, REG_T_ON_2, REG_T_ON_3, REG_T_ON_4, REG_T_ON_5, REG_T_ON_6,
REG_T_ON_7, REG_T_ON_8, REG_T_ON_9, REG_T_ON_10, REG_T_ON_11, REG_T_ON_12, REG_T_ON_13, REG_T_ON_14, REG_T_ON_15 };

uint8_t _reg_off[16] = { REG_OFF_0, REG_OFF_1, REG_OFF_2, REG_OFF_3, REG_OFF_4, REG_OFF_5, REG_OFF_6, REG_OFF_7,
REG_OFF_8, REG_OFF_9, REG_OFF_10, REG_OFF_11, REG_OFF_12, REG_OFF_13, REG_OFF_14, REG_OFF_15 };

uint8_t _reg_t_rise[16] = { 0xFF, 0xFF, 0xFF, 0xFF, REG_T_RISE_4, REG_T_RISE_5, REG_T_RISE_6, REG_T_RISE_7, 0xFF, 0xFF,
    0xFF, 0xFF, REG_T_RISE_12, REG_T_RISE_13, REG_T_RISE_14, REG_T_RISE_15 };

uint8_t _reg_t_fall[16] = { 0xFF, 0xFF, 0xFF, 0xFF, REG_T_FALL_4, REG_T_FALL_5, REG_T_FALL_6, REG_T_FALL_7, 0xFF, 0xFF,
    0xFF, 0xFF, REG_T_FALL_12, REG_T_FALL_13, REG_T_FALL_14, REG_T_FALL_15 };

SX1509::SX1509(const char* filename) :
    _filename(filename) {
}

SX1509::~SX1509() {
  close(_fd);
}

bool SX1509::isOpen() {
  return _open;
}

bool SX1509::_read16(uint8_t reg, uint16_t* value) {

  _txbuf[0] = reg;
  if (write(_fd, _txbuf, 1) != 1) {
    printf("Write failed, aborting read.\n\r");
    return false;
  } else {
    if (read(_fd, _rxbuf, 2) != 2) {
      printf("Read failed, aborting read.\n\r");
      return false;
    }
  }

  *value = _rxbuf[0] + (_rxbuf[1] << 8);
  return true;
}

bool SX1509::_read8(uint8_t reg, uint8_t* value) {
  _txbuf[0] = reg;
  if (write(_fd, _txbuf, 1) != 1) {
    printf("Write failed, aborting read.\n\r");
    return false;
  } else {
    if (read(_fd, _rxbuf, 1) != 1) {
      printf("Read failed, aborting read.\n\r");
      return false;
    }
  }

  *value = _rxbuf[0];
  return true;
}

bool SX1509::_write16(uint8_t reg, uint16_t* value) {

  _txbuf[0] = reg;
  _txbuf[1] = (*value & 0xFF00) >> 8;
  _txbuf[2] = *value & 0x00FF;

  if (write(_fd, _txbuf, 3) != 3) {
    printf("Write16 on reg %d failed.", (int) reg);
    return false;
  } else {
    return true;
  }
}

bool SX1509::_write8(uint8_t reg, uint8_t* value) {

  _txbuf[0] = reg;
  _txbuf[1] = *value;

  if (write(_fd, _txbuf, 2) != 2) {
    printf("Write8 on reg %d failed.", (int) reg);
    return false;
  } else {
    return true;
  }
}

bool SX1509::_reset(void) {
  uint16_t reset_code = (0x12 << 8) | 0x34;
  return _write16( REG_RESET, &reset_code);
}

bool SX1509::init(void) {
  if ((_fd = open(_filename.c_str(), O_RDWR)) < 0) {
    // Something went wrong, exit 1
    perror("Failed to open the i2c bus");
    _open = false;
    return false;
  }

  _open = true;

  //ioctl to set address
  if (ioctl(_fd, I2C_SLAVE, SX1509_ADDRESS) < 0) {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    /* ERROR HANDLING */
    return false;
  }

  bool result = true;
  uint16_t ones = 0xFFEA;
  uint16_t zeroes = 0x0015;
  uint8_t temp8;
  //input buffer
  result = result && _write16( REG_INPUT_DISABLE_A, &ones);
  //pullup
  result = result && _write16( REG_PULL_UP_A, &zeroes);
  //open drain
  result = result && _write16( REG_OPEN_DRAIN_A, &ones);
  //pindir
  result = result && _write16( REG_DIR_A, &zeroes);
  //Enable oscillator clock
  result = result && _read8( REG_CLOCK, &temp8);
  temp8 |= (1 << 6);
  temp8 &= ~(1 << 5);
  result = result && _write8( REG_CLOCK, &temp8);

  //Configure LED driver clock and mode
  result = result && _read8( REG_MISC, &temp8);
  temp8 &= ~(1 << 7); //Setting linear mode for IO bank A
  temp8 &= ~(1 << 3); //Setting linear mode for IO bank B

  temp8 |= 0x70; //Setting LED Clock to maximum (arbitrarily)

  result = result && _write8( REG_MISC, &temp8);

  result = result && _write16( REG_LED_DRIVER_ENABLE_A, &ones);

  result = result && _write16( REG_DATA_A, &zeroes);

  return result;
}

bool SX1509::pwm(uint8_t pin, uint8_t intensity) {
  return _write8(_reg_i_on[pin], &intensity);
}

bool SX1509::blink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t offIntensity, uint8_t onIntensity, uint8_t tRise,
    uint8_t tFall) {

  bool result = true;

  tOn &= 0x1F; // Cut tOn to 5 bit value
  tOff &= 0x1F; // Cut tOff to 5 bit value

  offIntensity &= 0x07; // Cut offIntensity to 3 bits.

  result = result && _write8(_reg_t_on[pin], &tOn);
  uint8_t reg_off = (tOff << 3) | offIntensity;
  result = result && _write8(_reg_off[pin], &reg_off);
  result = result && _write8(_reg_i_on[pin], &onIntensity);

  tRise &= 0x1F; // Cut tRise to 5 bits
  tFall &= 0x1F; // Cut tFall to 5 bits

  if (_reg_t_rise[pin] != 0xFF) { //Check that this is a valid breathe/blink pin
    result = result && _write8(_reg_t_rise[pin], &tRise);
  }

  if (_reg_t_fall[pin] != 0xFF) { //Check that this is a valid breathe/blink pin
    result = result && _write8(_reg_t_fall[pin], &tFall);
  }

  return result;
}
