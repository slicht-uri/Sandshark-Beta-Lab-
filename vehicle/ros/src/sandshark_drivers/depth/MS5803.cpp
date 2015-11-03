#include "MS5803.h"

MS5803::MS5803(const char* filename, int addr) {
  _addr = addr;
  _open = true;
  if ((_fd = open(filename, O_RDWR)) < 0) {
    // Something went wrong, exit 1
    perror("Failed to open the i2c bus");
    _open = false;
  } else {
    Reset();
    usleep(1000);
    Begin();
    usleep(1000);
    ReadProm();
    usleep(1000);
  }

}

MS5803::~MS5803() {
  close(_fd);
}

bool MS5803::isOpen() {
  return _open;
}

bool MS5803::Begin() {
  if (ioctl(_fd, I2C_SLAVE, _addr) < 0) {
    //Failed to initialize communications with the sensor
    printf("Failed to begin communications.\n");
    return false;
  }
  return true;
}

bool MS5803::Reset() {
  _txbuf[0] = ms5803_reset;
  if (!Begin()) {
    return false;
  }
  if (write(_fd, _txbuf, 1) != 1) {
    printf("Failed to write Reset command\n");
    return false;
  }
  return true;
}

bool MS5803::ReadProm(void) {
  int i, j;
  //initialize communications with sensor
  if (!Begin()) {
    return false;
  }
  for (i = 0; i < 8; i++) {
    j = i;
    _txbuf[0] = ms5803_PROMread + (j << 1);
    //Write ReadPROM command for PROM register
    if (write(_fd, _txbuf, 1) != 1) {
      printf("Failed to write ReadPROM command\n");
    }
    //Read PROM value
    if (read(_fd, _rxbuf, 2) != 2) {
      printf("Failed to read ReadPROM return value\n");
    } else {
      _prom[i] = _rxbuf[1] + (_rxbuf[0] << 8);
    }
  }
  //copy prom values 1-6 into _calib
  for (i = 1; i < 7; i++) {
    _calib[i - 1] = _prom[i];
  }
  return true;
}

int MS5803::ReadAdc(void) {
  _txbuf[0] = ms5803_ADCread;
  int adcvalue;
  //Initialize communications with sensor
  Begin();
  //write ADCread command to sensor
  if (write(_fd, _txbuf, 1) != 1) {
    printf("Failed to read ADC\n");
    return -1;
  }
  //read value from sensor
  if (read(_fd, _rxbuf, 3) != 3) {
    printf("Failed to read ADC\n");
    return -1;
  }
  //convert 3 byte return value to int
  adcvalue = _rxbuf[2] + (_rxbuf[1] << 8) + (_rxbuf[0] << 16);
  return adcvalue;
}

void MS5803::ConvertD1(void) {
  _txbuf[0] = ms5803_convD1_4096;
  //Initialize communications with sensor
  Begin();
  //write ConvertD1 command to sensor
  if (write(_fd, _txbuf, 1) != 1) {
    printf("Failed to write convertD1 command\n");
  }
}

void MS5803::ConvertD2(void) {
  _txbuf[0] = ms5803_convD2_4096;
  //Initialize communications with sensor
  Begin();
  //write ConvertD2 command to sensor
  if (write(_fd, _txbuf, 1) != 1) {
    printf("Failed to write convertD2 command\n");
  }
}

void MS5803::Read(float* pressure, float* temperature) {
  //Send command to collect pressure data
  ConvertD1();
  //Wait for collection to finish
  usleep(10000);
  //Read ADC to get pressure value
  int raw_pressure = ReadAdc();

  //Send command to collect temperature data
  ConvertD2();
  //Wait for collection to finish
  usleep(10000);
  //Read ADC to get temperature data
  int raw_temperature = ReadAdc();

  //Calculate Temperature in degrees C
  long dT;
  float calib_temperature;
  dT = raw_temperature - _calib[4] * pow(2, 8);
  calib_temperature = 20 + dT * _calib[5] / (pow(2, 23) * 100);

  //Calculate Pressure in mBar
  float pressure_offset, pressure_sensitivity, calib_pressure;
  pressure_offset = _calib[1] * pow(2, 16) + _calib[3] * dT / pow(2, 7);
  pressure_sensitivity = _calib[0] * pow(2, 15) + _calib[2] * dT / pow(2, 8);
  calib_pressure = (raw_pressure * pressure_sensitivity / pow(2, 21) - pressure_offset) / (pow(2, 13) * 10);

  *temperature = calib_temperature; //temperature in degrees C
  *pressure = calib_pressure; //pressure in mBar

}
