#include "lib_bme280.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <errno.h>

#define DEVICE_PATH "/dev/bme280"

// ---------------------------------------DEFINE IOCTL --------------------------------
#define BME280_IOCTL_MAGIC 'm'
#define BME280_READ_TEMP   _IOR(BME280_IOCTL_MAGIC, 1, int)
#define BME280_READ_PRESS  _IOR(BME280_IOCTL_MAGIC, 2, int)
#define BME280_READ_HUM    _IOR(BME280_IOCTL_MAGIC, 3, int)
#define BME280_SET_CONFIG  _IOW(BME280_IOCTL_MAGIC, 4, struct bme280_config)
#define BME280_WAKEUP      _IOR(BME280_IOCTL_MAGIC, 5, int)
#define BME280_STATUS      _IOR(BME280_IOCTL_MAGIC, 6, int)

// +++++++++++++++++++++++++++++++++++++ CAC HAM CUA THU VIEN ++++++++++++++++++++++++++++++++++++
// ------------------------------------- HAM OPEN DRIVER ----------------------------------------
int bme280_open()
{
  int fd = open(DEVICE_PATH, O_RDWR);
  if (fd < 0)
  {
    perror("Failed to open the device");
    return -1;
  }
  return fd;
}
// -------------------------------------HAM CLOSE ----------------------------------------
int bme280_close(int fd)
{
  if (close(fd) < 0)
  {
    perror("Failed to close the device");
    return -1;
  }
  return 0;
}
// -----------------------------------HAM DOC GIA TRI NHIET DO ----------------------------
float bme280_read_temp(int fd)
{
  int temp;
  if (ioctl(fd, BME280_READ_TEMP, &temp) < 0)
  {
    perror("Failed to read temperature");
    return -1.0;
  }
  return temp / 100.0;
}
//-----------------------------------HAM DOC GIA TRI DO AM --------------------------------
float bme280_read_hum(int fd)
{
  int hum;
  if (ioctl(fd, BME280_READ_HUM, &hum) < 0)
  {
    perror("Failed to read humidity");
    return -1.0;
  }
  return hum / 1024.0;
}
//----------------------------------HAM DOC GIA TRI AP SUAT ---------------------------------
float bme280_read_press(int fd)
{
  int press;
  if (ioctl(fd, BME280_READ_PRESS, &press) < 0)
  {
    perror("Failed to read pressure");
    return -1.0;
  }
  return press / 1.0;
}
//----------------------------------HAM CAU HINH CHO BME280 ----------------------------------
int bme280_config(int fd, int osrsT, int osrsP, int osrsH, int MOde, int Tsb, int IIR)
{

  struct bme280_config config;
  config.osrs_t = osrsT;
  config.osrs_p = osrsP;
  config.osrs_h = osrsH;
  config.mode   = MOde;
  config.t_sb   = Tsb;
  config.filter = IIR;

  if (ioctl(fd, BME280_SET_CONFIG, &config) < 0)
  {
    perror("Failed to set config");
    return -1;
  }
  return 0;
}
//---------------------------------HAM WAKEUP ----------------------------------------------
int bme280_wakeup(int fd)
{
  if (ioctl(fd, BME280_WAKEUP) < 0)
  {
    perror("No Wakeup");
    return -1;
  }
  return 0;
}
//--------------------------------STATUS BME280 --------------------------------------------
int bme280_status(int fd)
{
  int sts;
  if (ioctl(fd, BME280_READ_HUM, &sts) < 0)
  {
    perror("Failed to read status");
    return -1.0;
  }
  if ((sts & 0x01) == 1)
    printf("the NVM data are being copied to image registers\n");
  else
    printf("the NVM data were copied to image registers (done)\n");

  if ((sts & 0x04) == 4)
    printf("A conversion is running\n");
  else
    printf("The results has been transferred to the data registers\n");

  return 0;
}
