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

#define MS5803_RX_DEPTH 3 // reads  are up to 3 bytes long
#define MS5803_TX_DEPTH 2 // writes are up to 2 bytes long

#define ms5803_addrCL 0x77 // CSB Pin Low  address
#define ms5803_addrCH 0x76 // CSB Pin High address

#define ms5803_reset  0x1E // Sensor Reset Command

#define ms5803_convD1_256  0x40 // Convert D1 OSR 256  Command
#define ms5803_convD1_512  0x42 // Convert D1 OSR 512  Command
#define ms5803_convD1_1024 0x44 // Convert D1 OSR 1024 Command
#define ms5803_convD1_2048 0x46 // Convert D1 OSR 2048 Command
#define ms5803_convD1_4096 0x48 // Convert D1 OSR 4096 Command

#define ms5803_convD2_256  0x50 // Convert D2 OSR  256 Command
#define ms5803_convD2_512  0x52 // Convert D2 OSR  512 Command
#define ms5803_convD2_1024 0x54 // Convert D2 OSR 1024 Command
#define ms5803_convD2_2048 0x56 // Convert D2 OSR 2048 Command
#define ms5803_convD2_4096 0x58 // Convert D2 OSR 4096 Command

#define ms5803_ADCread     0x00 // read ADC command
#define ms5803_PROMread    0xA0 // read PROM command base address

struct MS5803_output {
    float pressure;
    float temperature;
};

class MS5803 {
  private:
    unsigned long _d1, _d2;
    int _fd;
    bool _open;
    int _addr;
    int _prom[8];
    int _calib[6];
    char _txbuf[MS5803_TX_DEPTH];
    char _rxbuf[MS5803_RX_DEPTH];
    bool Begin();
    bool Reset();
    bool ReadProm();
    int ReadAdc();
    void ConvertD1();
    void ConvertD2();

  public:
    MS5803(const char* filename, int addr);
    ~MS5803();
    bool isOpen();
    void Read(float*, float*);
};
