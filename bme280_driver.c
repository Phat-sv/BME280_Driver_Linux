#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "bme280_driver"
#define CLASS_NAME  "bme280"
#define DEVICE_NAME "bme280"

#define BME280_ADDRESS 0x76
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

// IOCTL commands
#define BME280_IOCTL_MAGIC 'm'
#define BME280_READ_TEMP   _IOR(BME280_IOCTL_MAGIC, 1, int)
#define BME280_READ_PRESS  _IOR(BME280_IOCTL_MAGIC, 2, int)
#define BME280_READ_HUM    _IOR(BME280_IOCTL_MAGIC, 3, int)
#define BME280_SET_CONFIG  _IOW(BME280_IOCTL_MAGIC, 4, struct bme280_config)
#define BME280_WAKEUP      _IOR(BME280_IOCTL_MAGIC, 5, int)
#define BME280_STATUS      _IOR(BME280_IOCTL_MAGIC, 6, int)

struct bme280_config
{
  int osrs_t;
  int osrs_p;
  int osrs_h;
  int mode;
  int t_sb;
  int filter;
};

static struct i2c_client *bme280_client;
static struct class      *bme280_class  = NULL;
static struct device     *bme280_device = NULL;
static int                major_number;
s32                       traw, praw, hraw;
s32                       t_fine;
u8                        chipID;

float Temperature, Pressure, Humidity;

u32 dig_T1, dig_P1, dig_H1, dig_H3;

s32 dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5,
  dig_H6;

// ----------------------------------------------- HAM DOC VA SAP XEP DU LIEU CALIP TU CM BIEN
// -----------------------------------
static void readTrim(void)
{
  u8 trimdata[32];
  // Read NVM from 0x88 to 0xA1
  i2c_smbus_read_i2c_block_data(bme280_client, 0x88, 25, trimdata);
  // Read NVM from 0xE1 to 0xE7
  i2c_smbus_read_i2c_block_data(bme280_client, 0xE1, 7, (u8 *)trimdata + 25);
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

// -------------------------------------------HAM DOC GIA TRI TU CAM BIEN BME280
// ------------------------------------
s32 BMEReadRaw(void)
{
  u8 RawData[8];
  // Check the chip ID before reading
  chipID = i2c_smbus_read_byte_data(bme280_client, ID_REG);

  if (chipID == 0x60)
  {
    // Read the Registers 0xF7 to 0xFE
    i2c_smbus_read_i2c_block_data(bme280_client, PRESS_MSB_REG, 8, RawData);

    praw = (RawData[0] << 12) | (RawData[1] << 4) | (RawData[2] >> 4);
    traw = (RawData[3] << 12) | (RawData[4] << 4) | (RawData[5] >> 4);
    hraw = (RawData[6] << 8) | (RawData[7]);

    return 0;
  }

  else
    return -1;
}
// ---------------------------------------------HAM TINH GIA TRI CUA NHIET
// --------------------------------------------
s32 read_temp(s32 traw)
{
  s32 var1, var2;
  /* Calculate temperature in degree */
  var1 = ((((traw >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11;
  var2 = (((((traw >> 4) - (dig_T1)) * ((traw >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14;
  return ((var1 + var2) * 5 + 128) >> 8;
}
// -------------------------------------------------HAM TINH GIA TRI DO AM
// ----------------------------------------------
s32 read_hum(int hraw)
{
  s32 var1;
  var1 = (t_fine - ((s32)76800));
  var1 = (((((hraw << 14) - (((s32)dig_H4) << 20) - (((s32)dig_H5) * var1)) + ((s32)16384)) >> 15) *
          (((((((var1 * ((s32)dig_H6)) >> 10) * (((var1 * ((s32)dig_H3)) >> 11) + ((s32)32768))) >> 10) +
             ((s32)2097152)) *
              ((s32)dig_H2) +
            8192) >>
           14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((s32)dig_H1)) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);
  return (u32)(var1 >> 12);
  return 1;
}

// -----------------------------------------------------HAM TINH GIA TRI AP SUAT
// -------------------------------------------------
s32 read_press(int praw)
{
  s32 var1, var2, p;
  var1 = (((s32)t_fine) >> 1) - (s32)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((s32)dig_P6);
  var2 = var2 + ((var1 * ((s32)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((s32)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((s32)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((s32)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0;  // avoid exception caused by division by zero
  }
  p = (((u32)(((s32)1048576) - praw) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((u32)var1);
  }
  else
  {
    p = (p / (u32)var1) * 2;
  }
  var1 = (((s32)dig_P9) * ((s32)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((s32)(p >> 2)) * ((s32)dig_P8)) >> 13;
  p    = (u32)((s32)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}
//--------------------------------------------+++++++++++++++++++++++++++++++++++++++++++++++++++-------------------------------------
void BME280_WakeUP(void)
{
  uint8_t datatowrite = 0;
  // first read the register
  datatowrite = i2c_smbus_read_byte_data(bme280_client, CTRL_MEAS_REG);
  // modify the data with the forced mode
  datatowrite = datatowrite | MODE_FORCED;
  // write the new data to the register
  i2c_smbus_write_byte_data(bme280_client, CTRL_MEAS_REG, datatowrite);
}
void resetBME280(void)
{
  // Soft reset bme280
  i2c_smbus_write_byte_data(bme280_client, RESET_REG, 0xB6);
}
s32 BMEReadStatus(void)
{
  int sts = 0;
  sts     = i2c_smbus_read_byte_data(bme280_client, STATUS_REG);
  return sts;
}
// ---------------------------------------HAM CONFIG CHO CAM BIEN BME280
// ------------------------------------------------------------
s32 BME280_Config(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
  u8 datatowrite = 0;
  // write the humidity oversampling to 0xF2
  datatowrite = osrs_h;
  i2c_smbus_write_byte_data(bme280_client, CTRL_HUM_REG, datatowrite);
  // write the standby time and IIR filter coeff to 0xF5
  datatowrite = (t_sb << 5) | (filter << 2);
  // datatowrite = (t_sb <<5);
  i2c_smbus_write_byte_data(bme280_client, CONFIG_REG, datatowrite);
  // write the pressure and temp oversampling along with mode to 0xF4
  datatowrite = (osrs_t << 5) | (osrs_p << 2) | mode;
  i2c_smbus_write_byte_data(bme280_client, CTRL_MEAS_REG, datatowrite);
  return 0;
}

// -----------------------------------------------------HAM IOCTL GIAO TIEP VOI USER
// -------------------------------------------------
static long bme280_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  int                  value;
  struct bme280_config config;
  readTrim();
  BMEReadRaw();
  switch (cmd)
  {
  case BME280_SET_CONFIG:
    if (copy_from_user(&config, (struct bme280_config __user *)arg, sizeof(config)))
    {
      return -EFAULT;
    }
    BME280_Config(config.osrs_t, config.osrs_p, config.osrs_h, config.mode, config.t_sb, config.filter);
    break;
  case BME280_READ_TEMP:
    value = read_temp(traw);
    if (copy_to_user((int __user *)arg, &value, sizeof(value)))
    {
      return -EFAULT;
    }
    break;
  case BME280_READ_PRESS:
    value = read_press(praw);
    if (copy_to_user((int __user *)arg, &value, sizeof(value)))
    {
      return -EFAULT;
    }
    break;
  case BME280_READ_HUM:
    value = read_hum(hraw);
    if (copy_to_user((int __user *)arg, &value, sizeof(value)))
    {
      return -EFAULT;
    }
    break;
  case BME280_WAKEUP:
    BME280_WakeUP();
    break;
  case BME280_STATUS:
    BMEReadStatus();
    break;
  default:
    return -ENOTTY;
  }

  return 0;
}

// ---------------------------------------------------------CAC HAM KHOI TAO DRIVER
// -------------------------------------

static int bme280_open(struct inode *inodep, struct file *filep)
{
  printk(KERN_INFO "bme280 device opened\n");
  return 0;
}

static int bme280_release(struct inode *inodep, struct file *filep)
{
  printk(KERN_INFO "bme280 device closed\n");
  return 0;
}
// -----------------------------------------------HAM XUAT DU LIEU USER
// -----------------------------------------------
static struct file_operations fops = {
  .open           = bme280_open,
  .unlocked_ioctl = bme280_ioctl,
  .release        = bme280_release,
};

static int bme280_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  bme280_client = client;

  // Create a char device
  major_number = register_chrdev(0, DEVICE_NAME, &fops);
  if (major_number < 0)
  {
    printk(KERN_ERR "Failed to register a major number\n");
    return major_number;
  }

  bme280_class = class_create(THIS_MODULE, CLASS_NAME);
  if (IS_ERR(bme280_class))
  {
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_ERR "Failed to register device class\n");
    return PTR_ERR(bme280_class);
  }

  bme280_device = device_create(bme280_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
  if (IS_ERR(bme280_device))
  {
    class_destroy(bme280_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_ERR "Failed to create the device\n");
    return PTR_ERR(bme280_device);
  }

  printk(KERN_INFO "bme280 driver installed\n");
  return 0;
}

static void bme280_remove(struct i2c_client *client)
{
  device_destroy(bme280_class, MKDEV(major_number, 0));
  class_unregister(bme280_class);
  class_destroy(bme280_class);
  unregister_chrdev(major_number, DEVICE_NAME);

  printk(KERN_INFO "bme280 driver removed\n");
}

static const struct of_device_id bme280_of_match[] = {
  {
    .compatible = "invensense,bme280",
  },
  {},
};
MODULE_DEVICE_TABLE(of, bme280_of_match);

static struct i2c_driver bme280_driver = {
  .driver =
    {
      .name           = DRIVER_NAME,
      .owner          = THIS_MODULE,
      .of_match_table = of_match_ptr(bme280_of_match),
    },
  .probe  = bme280_probe,
  .remove = bme280_remove,
};

// ----------------------------------------------HAM KHOI TAO BME280 -----------------------------------------
static int __init bme280_init(void)
{
  printk(KERN_INFO "Initializing bme280 driver\n");
  return i2c_add_driver(&bme280_driver);
}
// ----------------------------------------------HAM XOA DRIVER BME280
// -------------------------------------------------
static void __exit bme280_exit(void)
{
  printk(KERN_INFO "Exiting bme280 driver\n");
  i2c_del_driver(&bme280_driver);
}
// -----------------------------------------------------KHAI BAO HAM
// --------------------------------------------
module_init(bme280_init);
module_exit(bme280_exit);
MODULE_AUTHOR("BME280");
MODULE_DESCRIPTION("bme280 I2C Client Driver with IOCTL Interface");
MODULE_LICENSE("GPL");
