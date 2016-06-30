#include "MS8607BA01.h"
#include <string.h>

MS8607BA01::MS8607BA01( int addr ) :
    _addr(addr) {
    _humidaddr = 0x40;
}

bool MS8607BA01::init( int fd ) {
  if( !ReadProm( fd ) ) {
    return false;
  }
  usleep(1000);

  return true;
}

MS8607BA01::~MS8607BA01() {}

bool MS8607BA01::Begin( int fd, bool humid ) {
  if (ioctl(fd, I2C_SLAVE, ( humid ? _humidaddr : _addr ) ) < 0) {
    //Failed to initialize communications with the sensor
    printf("Failed to begin communications.\n");
    return false;
  }

  usleep( 10000 );
  return true;
}

bool MS8607BA01::ReadProm( int fd ) {
  //initialize communications with sensor
  if( !Begin( fd ) ) {
    return false;
  }
  usleep(1000);

  printf("envPROM: ");

  for( int i = 0; i < 6; i++ ) {
    int j = i;
    _txbuf[0] = MS8607BA01_RA_BASE + (j << 1);
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
 
  _c1 = _prom[0];
  _c2 = _prom[1];
  _c3 = _prom[2];
  _c4 = _prom[3];
  _c5 = _prom[4];
  _c6 = _prom[5];

  printf("ENV c1: %d, c2: %d, c3: %d, c4: %d, c5: %d, c6: %d.\n", _c1, _c2, _c3, _c4, _c5, _c6);

  return true;
}

int MS8607BA01::ReadAdc( int fd, bool humid ) {
  _txbuf[0] = ( humid ? 0xE5 : MS8607BA01_ADCread );
  int adcvalue;
  //Initialize communications with sensor
  Begin( fd, humid );
  //write ADCread command to sensor
  if( write( fd, _txbuf, 1 ) != 1 ) {
    printf("ENV Failed to read ADC is %s\n", (humid ? "humid" : "PT" ) );
    return -1;
  }
  int retbytes = 3;
  usleep(10000);

  if( humid ) { 
    retbytes = 2;
  }
  //read value from sensor
  if( read( fd, _rxbuf, retbytes ) != retbytes ) {
    printf("ENV2 Failed to read ADC is %s -- errno %d:%s\n", (humid ? "humid" : "PT" ), errno, strerror( errno )  );
    return -1;
  }
  if( humid ) {
    //convert 2 byte return value to int
    adcvalue = _rxbuf[1] + (_rxbuf[0] << 8);
  } else {
    //convert 3 byte return value to int
    adcvalue = _rxbuf[2] + (_rxbuf[1] << 8) + (_rxbuf[0] << 16);
  }
  return adcvalue;
}

bool MS8607BA01::ConvertD1( int fd ) {
  _txbuf[0] = MS8607BA01_convD1_4096;
  //Initialize communications with sensor
  Begin( fd );
  //write ConvertD1 command to sensor
  if( write( fd, _txbuf, 1) != 1 ) {
    printf("Failed to write convertD1 command\n");
    return false;
  }
  return true;
}

bool MS8607BA01::ConvertD2( int fd ) {
  _txbuf[0] = MS8607BA01_convD2_4096;
  //Initialize communications with sensor
  Begin( fd );
  //write ConvertD2 command to sensor
  if( write( fd, _txbuf, 1) != 1 ) {
    printf("ENV Failed to write convertD2 command\n");
    return false;
  }
  return true;
}

bool MS8607BA01::ConvertRH( int fd ) {
  _txbuf[0] = 0xFE;
  //Initialize communications with sensor
  Begin( fd, true );

  if( write( fd, _txbuf, 1) != 1 ) {
    printf("ENV Failed to write convertD2 command\n");
    return false;
  }
  
  return true;
}

bool MS8607BA01::Read( int fd, float* pressure, float* temperature, float *humidity ) {
  //Send command to collect pressure data
  if( !ConvertD1( fd ) ) {
    return false;
  }

  //Wait for collection to finish
  usleep(10000);
  //Read ADC to get pressure value
  int raw_pressure = ReadAdc( fd );

  //Send command to collect temperature data
  if( !ConvertD2( fd ) ) {
    return false;
  }

  //Wait for collection to finish
  usleep(10000);
  //Read ADC to get temperature data
  int raw_temperature = ReadAdc( fd );
 
  if( !ConvertRH( fd ) ){ 
    return false;
  }
  
  usleep(10000);

  int rh = ReadAdc( fd, true );
 
//  printf( "RawT = %d, RawP = %d RH = %d\n", raw_temperature, raw_pressure, rh );
  float dT = (float)raw_temperature - (((float)_c5) * ( 1 << 8 ));
  float TEMP = 2000.0 + dT * ((float)_c6)/( 1 << 23 );
  float OFF = (float)_c2 * ( 1 << 17 ) + ( (float)_c4 * dT )/( 1 << 6 );
  float SENS = (float)_c1 * ( 1 << 16 ) + ( (float)_c3 * dT )/( 1 << 7 );
//  printf( "dT %f TEMP %f, OFF %f, SENS %f\n", dT, TEMP, OFF, SENS );
  float T2 = 0.0;
  float OFF2 = 0.0;
  float SENS2 = 0.0;
  if( TEMP < 20.0 ) {
    T2 = ( dT * dT )/( pow( 2, 31 ) );
    OFF2 = 5.0 * ( TEMP - 2000.0 ) * ( TEMP - 2000.0 )/( 2.0 );
    SENS2 = OFF2/( 2.0 );
    if( TEMP < -1500.0 ) {
      OFF2 += 7.0 * ( TEMP + 1500.0 ) * ( TEMP + 1500.0 );
      SENS2 += 11.0 * ( TEMP + 1500.0 ) * ( TEMP + 1500.0 )/2;
    } 
  }
//  printf( "T2 %f, OFF2 %f SENS2 %f\n", T2, OFF2, SENS2 );

  TEMP -= T2;
  OFF -= OFF2;
  SENS -= SENS2;
  
  *temperature = TEMP/100.0;
  *pressure = ( raw_pressure * ( SENS/(1 << 21) ) - OFF )/( 1 << 15 );
  *humidity = ( -6.0 + 125.0 * ( ((float)rh)/ ( 1 << 16 ) ) );

  return true;
}


