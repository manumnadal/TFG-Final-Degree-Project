#ifndef __LCD_1602A__
#define __LCD_1602A__

#include <zephyr/device.h>

void sendCMD(const struct device *dev, unsigned char cmd);
void sendData(const struct device *dev, unsigned char data);
void sendString(const struct device * dev, char *str);
void pcf857_LCDinit(const struct device * dev);
void pcf857_Clear(const struct device *dev);
void pcf857_LCDGOTO(const struct device * dev);

#endif