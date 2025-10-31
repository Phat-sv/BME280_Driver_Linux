// test bme280
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdint.h>  // int16_t
#include <math.h>    // gcc ....-lwiringPi -lm
#include <stdlib.h>
#include <string.h>

#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>  // need -li2c when compile
                        // sudo apt install libi2c-dev

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

// REGISTERS DEFINITIONS
#define ID_REG        0xD0
#define RESET_REG     0xE0
#define CTRL_HUM_REG  0xF2
#define STATUS_REG    0xF3
#define CTRL_MEAS_REG 0xF4
#define CONFIG_REG    0xF5
#define PRESS_MSB_REG 0xF7

int bme;

uint8_t chipID;

uint8_t TrimParam[36];
int32_t tRaw, pRaw, hRaw;

uint16_t dig_T1, dig_P1, dig_H1, dig_H3;

int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int16_t dig_H2, dig_H4, dig_H5, dig_H6;

// Thiet lap I2C
void setupI2C(void)
{
  // load I2C driver
  bme = open("/dev/i2c-1", O_RDWR);
  if (bme < 0)
  {
    printf("Can't load I2C driver\n");
    exit(1);
  }

  // set slave address
  int addr = 0x76;
  if (ioctl(bme, I2C_SLAVE, addr) < 0)
  {
    printf("Can't connect to the device at: 0x76\n");
    exit(1);
  }
}

// Read the Trimming parameters saved in the NVM ROM of the device
void TrimRead(void)
{
  uint8_t trimdata[32];

  // Read NVM from 0x88 to 0xA1
  i2c_smbus_read_i2c_block_data(bme, 0x88, 25, trimdata);

  // Read NVM from 0xE1 to 0xE7
  i2c_smbus_read_i2c_block_data(bme, 0xE1, 7, (uint8_t *)trimdata + 25);

  // Arrange the data as per the datasheet (page no. 24)
  dig_T1 = (trimdata[1] << 8) | trimdata[0];
  dig_T2 = (trimdata[3] << 8) | trimdata[2];
  dig_T3 = (trimdata[5] << 8) | trimdata[4];
  dig_P1 = (trimdata[7] << 8) | trimdata[5];
  dig_P2 = (trimdata[9] << 8) | trimdata[6];
  dig_P3 = (trimdata[11] << 8) | trimdata[10];
  dig_P4 = (trimdata[13] << 8) | trimdata[12];
  dig_P5 = (trimdata[15] << 8) | trimdata[14];
  dig_P6 = (trimdata[17] << 8) | trimdata[16];
  dig_P7 = (trimdata[19] << 8) | trimdata[18];
  dig_P8 = (trimdata[21] << 8) | trimdata[20];
  dig_P9 = (trimdata[23] << 8) | trimdata[22];
  dig_H1 = trimdata[24];
  dig_H2 = (trimdata[26] << 8) | trimdata[25];
  dig_H3 = (trimdata[27]);
  dig_H4 = (trimdata[28] << 4) | (trimdata[29] & 0x0f);
  dig_H5 = (trimdata[30] << 4) | (trimdata[29] >> 4);
  dig_H6 = (trimdata[31]);
}

void resetBME280(void)
{
  // Soft reset bme280
  i2c_smbus_write_byte_data(bme, RESET_REG, 0xB6);
}

int BME280_Config(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
  // Read the Trimming parameters
  TrimRead();

  // Variables to write and check
  uint8_t datatowrite = 0;
  uint8_t datacheck   = 0;

  // Reset the device
  resetBME280();

  // write the humidity oversampling to 0xF2
  datatowrite = osrs_h;
  i2c_smbus_write_byte_data(bme, CTRL_HUM_REG, datatowrite);

  // write the standby time and IIR filter coeff to 0xF5
  datatowrite = (t_sb << 5) | (filter << 2);
  i2c_smbus_write_byte_data(bme, CONFIG_REG, datatowrite);

  // write the pressure and temp oversampling along with mode to 0xF4
  datatowrite = (osrs_t << 5) | (osrs_p << 2) | mode;
  i2c_smbus_write_byte_data(bme, CTRL_MEAS_REG, datatowrite);

  return 0;
}

void BMEReadStatus(void)
{
  int sts = 0;
  sts     = i2c_smbus_read_byte_data(bme, STATUS_REG);

  if ((sts & 0x01) == 1)
    printf("the NVM data are being copied to image registers\n");
  else
    printf("the NVM data were copied to image registers (done)\n");

  if ((sts & 0x04) == 4)
    printf("A conversion is running\n");
  else
    printf("The results has been transferred to the data registers\n");

  printf("//////////////////////////////////////////////////////\n");
}

int BMEReadRaw(void)
{
  uint8_t RawData[8];

  // Check the chip ID before reading
  chipID = i2c_smbus_read_byte_data(bme, ID_REG);

  if (chipID == 0x60)
  {
    // Read the Registers 0xF7 to 0xFE
    i2c_smbus_read_i2c_block_data(bme, PRESS_MSB_REG, 8, RawData);

    /* Calculate the Raw data for the parameters
     * Here the Pressure and Temperature are in 20 bit format and humidity in 16 bit format
     */
    pRaw = (RawData[0] << 12) | (RawData[1] << 4) | (RawData[2] >> 4);
    tRaw = (RawData[3] << 12) | (RawData[4] << 4) | (RawData[5] >> 4);
    hRaw = (RawData[6] << 8) | (RawData[7]);

    return 0;
  }

  else
    return -1;
}

void BME280_WakeUP(void)
{
  uint8_t datatowrite = 0;

  // first read the register
  datatowrite = i2c_smbus_read_byte_data(bme, CTRL_MEAS_REG);

  // modify the data with the forced mode
  datatowrite = datatowrite | MODE_FORCED;

  // write the new data to the register
  i2c_smbus_write_byte_data(bme, CTRL_MEAS_REG, datatowrite);

  usleep(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////// ***Function to compensate value*** //////////////////////////////////////////
int32_t t_fine;

int32_t BME280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 =
    (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >>
    14;
  t_fine = var1 + var2;
  T      = (t_fine * 5 + 128) >> 8;
  return T;
}

uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0)
  {
    return 0;  // avoid exception caused by division by zero
  }
  p    = 1048576 - adc_P;
  p    = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p    = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)p;
}

uint32_t BME280_compensate_H_int32(int32_t adc_H)
{
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r =
    (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >>
      15) *
     (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
          (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >>
         10) +
        ((int32_t)2097152)) *
         ((int32_t)dig_H2) +
       8192) >>
      14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r >> 12);
}

void BME280_Measure(float Temperature, float Pressure, float Humidity)
{
  if (BMEReadRaw() == 0)
  {
    if (tRaw == 0x800000)
      Temperature = 0;  // value in case temp measurement was disabled
    else
    {
      Temperature = (BME280_compensate_T_int32(tRaw)) / 100.0;  // as per datasheet, the temp is x100
      printf("Nhiet do: %f °C\n", Temperature);
    }

    if (pRaw == 0x800000)
      Pressure = 0;  // value in case temp measurement was disabled
    else
    {
      Pressure = (BME280_compensate_P_int64(pRaw)) / 256.0;  // as per datasheet, the pressure is x256
      printf("Ap suat: %f Pa\n", Pressure);
    }

    if (hRaw == 0x8000)
      Humidity = 0;  // value in case temp measurement was disabled
    else
    {
      Humidity = (BME280_compensate_H_int32(hRaw)) / 1024.0;  // as per datasheet, the temp is x1024
      printf("Do am: %f %%RH\n", Humidity);
    }
  }

  // if the device is detached
  else
  {
    Temperature = Pressure = Humidity = 0;
  }
}

int main(void)
{
  // Initialize variables
  float Temperature, Pressure, Humidity;
  setupI2C();
  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);

  while (1)
  {
    BME280_Measure(Temperature, Pressure, Humidity);
    usleep(1000000);
    printf("/////////////////////////////////////////////////\n");
    printf("\n");
  }

  return 0;
}
