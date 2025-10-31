#ifndef BME280_H
#define BME280_H

// Oversampling definitions
#define OSRS_OFF 0x00
#define OSRS_1   0x01
#define OSRS_2   0x02
#define OSRS_4   0x03
#define OSRS_8   0x04
#define OSRS_16  0x05

// MODE definitions
#define MODE_SLEEP  0x00
#define MODE_FORCED 0x01
#define MODE_NORMAL 0x03

// Standby time
#define T_SB_0p5  0x00
#define T_SB_62p5 0x01
#define T_SB_125  0x02
#define T_SB_250  0x03
#define T_SB_500  0x04
#define T_SB_1000 0x05
#define T_SB_10   0x06
#define T_SB_20   0x07

// IIR filter coefficients
#define IIR_OFF 0x00
#define IIR_2   0x01
#define IIR_4   0x02
#define IIR_8   0x03
#define IIR_16  0x04

struct bme280_config
{
  int osrs_t;
  int osrs_p;
  int osrs_h;
  int mode;
  int t_sb;
  int filter;
};

int   bme280_open();
int   bme280_close(int fd);
float bme280_read_temp(int fd);
float bme280_read_hum(int fd);
float bme280_read_press(int fd);
int   bme280_config(int fd, int osrsT, int osrsP, int osrsH, int MOde, int Tsb, int IIR);
int   bme280_wakeup(int fd);
int   bme280_status(int fd);

#endif
