#include <stdint.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <i2c-dev.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <string>

#define MS8607BA01_RX_DEPTH 3 // reads  are up to 3 bytes long
#define MS8607BA01_TX_DEPTH 2 // writes are up to 2 bytes long

#define MS8607BA01_RA_BASE 0xA2

#define MS8607BA01_ADCread     0x00 // read ADC command

#define MS8607BA01_convD1_256  0x40 // Convert D1 OSR 256  Command
#define MS8607BA01_convD1_512  0x42 // Convert D1 OSR 512  Command
#define MS8607BA01_convD1_1024 0x44 // Convert D1 OSR 1024 Command
#define MS8607BA01_convD1_2048 0x46 // Convert D1 OSR 2048 Command
#define MS8607BA01_convD1_4096 0x48 // Convert D1 OSR 4096 Command
#define MS8607BA01_convD1_8192 0x4A // Convert D1 OSR 8192 Command

#define MS8607BA01_convD2_256  0x50 // Convert D2 OSR  256 Command
#define MS8607BA01_convD2_512  0x52 // Convert D2 OSR  512 Command
#define MS8607BA01_convD2_1024 0x54 // Convert D2 OSR 1024 Command
#define MS8607BA01_convD2_2048 0x56 // Convert D2 OSR 2048 Command
#define MS8607BA01_convD2_4096 0x58 // Convert D2 OSR 4096 Command
#define MS8607BA01_convD2_8192 0x5A // Convert D2 OSR 8192 Command

class MS8607BA01 {
  private:
    unsigned long _d1, _d2;
    int _addr, _humidaddr;
    uint16_t _prom[6];
    uint16_t _c1, _c2, _c3, _c4, _c5, _c6;
    char _txbuf[MS8607BA01_TX_DEPTH];
    char _rxbuf[MS8607BA01_RX_DEPTH];
    bool Begin( int fd, bool humid = false );
    bool ReadProm( int fd );
    int ReadAdc( int fd, bool humid = false );
    bool ConvertD1( int fd );
    bool ConvertD2( int fd );
    bool ConvertRH( int fd );

  public:
    MS8607BA01( int addr );
    ~MS8607BA01();
    bool Read( int fd, float*, float*, float* );
    bool init( int fd );
};


