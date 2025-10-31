#include "lib_bme280.h"
#include <stdio.h>
#include <unistd.h>

int main()
{
  // Buoc 1:  ham open driver
  int fd = bme280_open();
  // Buoc 2:  ham config cho bme280
  bme280_config(fd, OSRS_16, OSRS_16, OSRS_16, MODE_NORMAL, T_SB_1000, IIR_16);

  while (1)
  {
    float temp  = bme280_read_temp(fd);
    float hum   = bme280_read_hum(fd);
    float press = bme280_read_press(fd);
    printf("Temperature: %.2f °C\n", temp);
    printf("Humidity: %.2f %%\n", hum);
    printf("Pressure: %.2f Pa\n", press);
    sleep(1);  // Delay 1 second
    printf("//////////////////////////////////////////////////////////\n");
  }

  bme280_close(fd);
  return 0;
}
