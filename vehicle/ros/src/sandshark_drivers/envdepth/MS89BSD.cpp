#include "MS89BSD.h"
MS89BSD::MS89BSD( int addr ) :
    _addr(addr) {
}

bool MS89BSD::init( int fd ) {
  if( !Reset( fd ) ) {
    return false;
  }

  usleep(1000);
  if( !ReadProm( fd ) ) {
    return false;
  }
  usleep(1000);

  return true;
}

MS89BSD::~MS89BSD() {}

bool MS89BSD::Begin( int fd ) {
  if (ioctl(fd, I2C_SLAVE, _addr) < 0) {
    //Failed to initialize communications with the sensor
    printf("Failed to begin communications.\n");
    return false;
  }
 
  usleep( 10000 );
  return true;
}

bool MS89BSD::Reset( int fd ) {
  _txbuf[0] = MS89BSD_reset;
  if( !Begin(fd) ) {
    return false;
  }
  usleep(1000);

  if (write( fd, _txbuf, 1 ) != 1) {
    printf("Failed to write Reset command\n");
    return false;
  }
  return true;
}

bool MS89BSD::ReadProm( int fd ) {
  int i, j;
  //initialize communications with sensor
  if( !Begin( fd ) ) {
    return false;
  }
  usleep(1000);

  printf("PROM: ");

  for (i = 0; i < 8; i++) {
    j = i;
    _txbuf[0] = MS89BSD_PROMread + (j << 1);
    //Write ReadPROM command for PROM register
    if( write( fd, _txbuf, 1 ) != 1) {
      printf("Failed to write ReadPROM command\n");
      return false;
    }
    //Read PROM value
    if( read( fd, _rxbuf, 2) != 2) {
      printf("Failed to read ReadPROM return value\n");
      return false;
    } else {
      _prom[i] = _rxbuf[1] + (_rxbuf[0] << 8);
      printf("%04X", _prom[i]);
    }
  }
  printf("\n\r");
  //Copy c parameters from appropriate places in _prom.
  //These are variable length signed ints, so we need to do
  //sign extension, then store them in int16's.
  _c0_tmp = (_prom[1] & 0xFFFC) >> 2;
  _c0 = signextend<signed int16_t, 14>(_c0_tmp);

  _c1_tmp = ((_prom[1] & 0x0003) << 12) + ((_prom[2] & 0xFFF0) >> 4);
  _c1 = signextend<signed int16_t, 14>(_c1_tmp);

  _c2_tmp = ((_prom[2] & 0x000F) << 6) + ((_prom[3] & 0xFC00) >> 10);
  _c2 = signextend<signed int16_t, 10>(_c2_tmp);

  _c3_tmp = (_prom[3] & 0x03FF);
  _c3 = signextend<signed int16_t, 10>(_c3_tmp);

  _c4_tmp = ((_prom[4] & 0xFFC0) >> 6);
  _c4 = signextend<signed int16_t, 10>(_c4_tmp);

  _c5_tmp = ((_prom[4] & 0x003F) << 4) + ((_prom[5] & 0xF000) >> 12);
  _c5 = signextend<signed int16_t, 10>(_c5_tmp);

  _c6_tmp = (_prom[5] & 0x0FFC) >> 2;
  _c6 = signextend<signed int16_t, 10>(_c6_tmp);

  //q is now a constant
  //Copy q parameters from appropriate places in memory.
  //These are all three bit  unsigned ints, so we can store them
  //directly in uint8's.
  //_q0 = ( _prom[5] & 0x001C ) >> 2;
  //_q1 = (( _prom[5] & 0x0003 ) << 1)  + (( _prom[6] & 0x8000 ) >> 15);
  //_q2 = ( _prom[6] & 0x7000 ) >> 12;
  _q0 = 9;
  _q1 = 11;
  _q2 = 9;
  _q3 = 15;
  _q4 = 15;
  _q5 = 16;
  _q6 = 16;

  //Copy a parameters from appropriate places in memory.
  //These are all ten bit signed ints, so we can store them
  //directly in int16's.
  _a0_tmp = ((_prom[5] & 0x0003) << 8) + ((_prom[6] & 0xFF00) >> 8);
  _a0 = (int16_t) signextend<int16_t, 10>(_a0_tmp);

  _a1_tmp = ((_prom[6] & 0x00FF) << 2) + ((_prom[7] & 0xC000) >> 14);
  _a1 = (int16_t) signextend<int16_t, 10>(_a1_tmp);

  _a2_tmp = (_prom[7] & 0x3FF0) >> 4;
  _a2 = (int16_t) signextend<int16_t, 10>(_a2_tmp);

  _crc = (_prom[7] & 0x000F);

  printf("c0: %d, c1: %d, c2: %d, c3: %d, c4: %d, c5: %d, c6: %d. q0: %d, q1: %d, q2: %d.  a0: %d, a1: %d, a2: %d\n\r",
      _c0, _c1, _c2, _c3, _c4, _c5, _c6, _q0, _q1, _q2, _a0, _a1, _a2);

  return true;
}

int MS89BSD::ReadAdc( int fd ) {
  _txbuf[0] = MS89BSD_ADCread;
  int adcvalue;
  //Initialize communications with sensor
  Begin( fd );
  //write ADCread command to sensor
  if( write( fd, _txbuf, 1 ) != 1 ) {
    printf("Failed to read ADC\n");
    return -1;
  }
  usleep(10000);
  //read value from sensor
  if( read( fd, _rxbuf, 3 ) != 3 ) {
    printf("Failed to read ADC\n");
    return -1;
  }
  //convert 3 byte return value to int
  adcvalue = _rxbuf[2] + (_rxbuf[1] << 8) + (_rxbuf[0] << 16);
  return adcvalue;
}

bool MS89BSD::ConvertD1( int fd ) {
  _txbuf[0] = MS89BSD_convD1_4096;
  //Initialize communications with sensor
  Begin( fd );
  //write ConvertD1 command to sensor
  if( write( fd, _txbuf, 1) != 1 ) {
    printf("Failed to write convertD1 command\n");
    return false;
  }
  return true;
}

bool MS89BSD::ConvertD2( int fd ) {
  _txbuf[0] = MS89BSD_convD2_4096;
  //Initialize communications with sensor
  Begin( fd );
  //write ConvertD2 command to sensor
  if( write( fd, _txbuf, 1) != 1 ) {
    printf("Failed to write convertD2 command\n");
    return false;
  }
  return true;
}

bool MS89BSD::Read( int fd, float* pressure, float* temperature ) {
  //Send command to collect pressure data
  if( !ConvertD1( fd ) ) {
    return false;
  }

  //Wait for collection to finish
  usleep(10000);
  //Read ADC to get pressure value
  int raw_pressure = ReadAdc( fd );

  float Pmax = 30.0; //Max Pressure is 30 Bar Absolute

  float Pmin = 0.0; // Min Pressure is 0 Bar Absolute

  //Send command to collect temperature data
  if( !ConvertD2( fd ) ) {
    return false;
  }

  //Wait for collection to finish
  usleep(10000);
  //Read ADC to get temperature data
  int raw_temperature = ReadAdc( fd );

  float scaled_temperature = raw_temperature / pow(2, 24);

  float calib_temperature = (((float) _a0) / 5.0)\
 + (((float) _a1) * 2.0 * scaled_temperature)\

      + (((float) _a2) * 2.0 * pow(scaled_temperature, 2));

  float Y_divisor = (raw_pressure\
 + (_c0 * pow(2, _q0)))\
 + (_c3 * pow(2, _q3) * scaled_temperature)\

      + (_c4 * pow(2, _q4) * pow(scaled_temperature, 2));

  float Y_dividend = (_c1 * pow(2, _q1))\
 + (_c5 * pow(2, _q5) * scaled_temperature)\

      + (_c6 * pow(2, _q6) * pow(scaled_temperature, 2));

  float Y = Y_divisor / Y_dividend;

  float P = Y * (1 - (_c2 * pow(2, _q2) / pow(2, 24))) + (_c2 * pow(2, _q2) / pow(2, 24) * pow(Y, 2));

  float calib_pressure = ((P - 0.1) / 0.8) * (Pmax - Pmin) + Pmin;

  /* float scale_temperature = raw_temperature/pow(2,24);

   // float calib_temperature = _a0*10 + (_a1*20*scale_temperature) + pow(_a2*20*scale_temperature,2);

   // float Y = (raw_pressure+_c0*pow(2,15-_q0))\
    //         + (_c3*pow(2,15)*scale_temperature)\
    //         + (_c4*pow(2,15)*pow(scale_temperature,2))/(_c1*pow(2,15-_q1) + _c5*pow(2,18)*scale_temperature + _c6*pow(2,18)*pow(scale_temperature,2));

   // float P = Y*(1-_c2*pow(2,15-_q2)/pow(2,24)) + _c2*pow(2,15-_q2)/(pow(2,24)*pow(Y,2));

   // float calib_pressure = ((P-0.1)/0.8)*(Pmax-Pmin) + Pmin;*/

  *temperature = calib_temperature; //temperature in degrees C
  *pressure = calib_pressure; //pressure in mBar

  return true;
}

template<typename T, unsigned B>
inline T signextend(const T x) {
  struct {
      T x :B;
  } s;
  return s.x = x;
}
