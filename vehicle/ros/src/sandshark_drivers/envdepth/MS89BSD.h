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

#define MS89BSD_RX_DEPTH 3 // reads  are up to 3 bytes long
#define MS89BSD_TX_DEPTH 2 // writes are up to 2 bytes long

#define MS89BSD_addrCL 0x77 // CSB Pin Low  address
#define MS89BSD_addrCH 0x76 // CSB Pin High address

#define MS89BSD_reset  0x1E // Sensor Reset Command

#define MS89BSD_convD1_256  0x40 // Convert D1 OSR 256  Command
#define MS89BSD_convD1_512  0x42 // Convert D1 OSR 512  Command
#define MS89BSD_convD1_1024 0x44 // Convert D1 OSR 1024 Command
#define MS89BSD_convD1_2048 0x46 // Convert D1 OSR 2048 Command
#define MS89BSD_convD1_4096 0x48 // Convert D1 OSR 4096 Command

#define MS89BSD_convD2_256  0x50 // Convert D2 OSR  256 Command
#define MS89BSD_convD2_512  0x52 // Convert D2 OSR  512 Command
#define MS89BSD_convD2_1024 0x54 // Convert D2 OSR 1024 Command
#define MS89BSD_convD2_2048 0x56 // Convert D2 OSR 2048 Command
#define MS89BSD_convD2_4096 0x58 // Convert D2 OSR 4096 Command

#define MS89BSD_ADCread     0x00 // read ADC command
#define MS89BSD_PROMread    0xA0 // read PROM command base address

struct MS89BSD_output {
    float pressure;
    float temperature;
};

class MS89BSD {
  private:
    unsigned long _d1, _d2;
    int _addr;
    uint16_t _prom[8];
    char _txbuf[MS89BSD_TX_DEPTH];
    char _rxbuf[MS89BSD_RX_DEPTH];
    uint16_t _c0_tmp, _c1_tmp, _c2_tmp, _c3_tmp, _c4_tmp, _c5_tmp, _c6_tmp;
    int16_t _c0, _c1, _c2, _c3, _c4, _c5, _c6;
    uint8_t _q0, _q1, _q2, _q3, _q4, _q5, _q6;
    uint16_t _a0_tmp, _a1_tmp, _a2_tmp;
    int16_t _a0, _a1, _a2;
    uint8_t _crc;
    bool Begin( int fd );
    bool Reset( int fd );
    bool ReadProm( int fd );
    int ReadAdc( int fd );
    bool ConvertD1( int fd );
    bool ConvertD2( int fd );

  public:
    MS89BSD( int addr );
    ~MS89BSD();
    bool Read( int fd, float*, float* );
    bool init( int fd );
};

template<typename T, unsigned B>
inline T signextend(const T x);
