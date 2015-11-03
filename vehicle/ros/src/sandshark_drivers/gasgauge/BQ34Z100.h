#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <i2c-dev.h>

#define BQ34Z100_SOC  0x02 // State of Charge command starts at 0x02 (0x02, 0x03)
#define BQ34Z100_RM   0x04 // Remaining Capacity command starts at 0x04 (0x04, 0x05)
#define BQ34Z100_FCC  0x06 // Full Charge Capacity command starts at 0x06 (0x06, 0x07)
#define BQ34Z100_VOLT 0x08 // Voltage command starts at 0x08 (0x08, 0x09)
#define BQ34Z100_AI   0x0A // Average Current command starts at 0x0A (0x0A, 0x0B)
#define BQ34Z100_TEMP 0x0C // Temperature command starts at 0x0C (0x0C, 0x0D)

#define BQ34Z100_ADDRESS 0x55 // Sensor Address is 0x55 (7 bit.  Read is 0xAA, Write is 0xAB)

class BQ34Z100 {
  private:
    char _txbuf[1];
    char _rxbuf[2];
    int _fd;
    bool _open;
    bool _read_value(int, int*);

    bool read_SOC(float*);
    bool read_RM(float*);
    bool read_FCC(float*);
    bool read_VOLT(float*);
    bool read_AI(float*);
    bool read_TEMP(float*);

  public:
    BQ34Z100();
    ~BQ34Z100();

    bool initialize(const char* filename);

    bool isOpen();
    bool Read(float*, float*, float*, float*, float*, float*);
};
