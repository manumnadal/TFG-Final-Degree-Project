#include "lcd1602a.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

uint8_t _LCDBackLight = 0x0F;


void sendCMD(const struct device * dev, unsigned char cmd){
    int error;

    // I2C MASK Byte = COMD-led-en-rw-rs (en=enable rs = reg select)(rw always write)
	const uint8_t LCDCmdByteOn = 0x0C;  // enable=1 and rs =0 1100 COMD-led-en-rw-rs 
	const uint8_t LCDCmdByteOff = 0x08; // enable=0 and rs =0 1000 COMD-led-en-rw-rs 
	
	unsigned char cmdNibbleLower, cmdNibbleUpper;
	uint8_t cmdBufferI2C[4];
	uint8_t I2CReturnCode = 0;
	
	cmdNibbleLower = (cmd << 4)&0xf0; //select lower nibble by moving it to the upper nibble position
	cmdNibbleUpper = cmd & 0xf0; //select upper nibble
	cmdBufferI2C[0] = cmdNibbleUpper | (LCDCmdByteOn & _LCDBackLight); // YYYY-1100 YYYY-led-en-rw-rs ,enable=1 and rs =0
	cmdBufferI2C[1] = cmdNibbleUpper | (LCDCmdByteOff & _LCDBackLight); // YYYY-1000 YYYY-led-en-rw-rs ,enable=0 and rs =0
	cmdBufferI2C[2] = cmdNibbleLower | (LCDCmdByteOn & _LCDBackLight); // YYYY-1100 YYYY-led-en-rw-rs ,enable=1 and rs =0
	cmdBufferI2C[3] = cmdNibbleLower | (LCDCmdByteOff & _LCDBackLight); // YYYY-1000 YYYY-led-en-rw-rs ,enable=0 and rs =0
	
	error = i2c_write(dev, cmdBufferI2C, sizeof(cmdBufferI2C), 0x27);
    if (error == 0) {
    }
    else {
        printk("error %d \n", error);
    }

}

void sendData(const struct device * dev, unsigned char data){
    int error;
    const uint8_t LCDDataByteOn= 0x0D; //enable=1 and rs =1 1101  DATA-led-en-rw-rs
    const  uint8_t LCDDataByteOff = 0x09; // enable=0 and rs =1 1001 DATA-led-en-rw-rs
    
    unsigned char dataNibbleLower, dataNibbleUpper;
    uint8_t dataBufferI2C[4];
    uint8_t I2CReturnCode = 0;

	dataNibbleLower = (data << 4)&0xf0; //select lower nibble by moving it to the upper nibble position
	dataNibbleUpper = data & 0xf0; //select upper nibble
	dataBufferI2C[0] = dataNibbleUpper | (LCDDataByteOn & _LCDBackLight); //enable=1 and rs =1 1101  YYYY-X-en-X-rs
	dataBufferI2C[1] = dataNibbleUpper | (LCDDataByteOff & _LCDBackLight); //enable=0 and rs =1 1001 YYYY-X-en-X-rs
	dataBufferI2C[2] = dataNibbleLower | (LCDDataByteOn & _LCDBackLight); //enable=1 and rs =1 1101  YYYY-X-en-X-rs
	dataBufferI2C[3] = dataNibbleLower | (LCDDataByteOff &  _LCDBackLight); //enable=0 and rs =1 1001 YYYY-X-en-X-rs

    error = i2c_write(dev, dataBufferI2C, sizeof(dataBufferI2C), 0x27);
    if (error == 0) {
    }
    else {
        printk("error %d \n", error);
    }

}

void sendString(const struct device * dev, char *str){
    while (*str) sendData(dev,*str++);
}

void pcf857_LCDinit(const struct device * dev){
    struct i2c_msg msgs[6];
    uint8_t HomePosition = 0x02;
    int error;

    for(int i = 0; i < 3; i++){
        
        printk("HomePosition: %d\n", i);
        sendCMD(dev, HomePosition);
    }

    uint8_t LCDModeFourBit = 0x28;

    printk("LCDModeFourBit\n");
        
    sendCMD(dev, LCDModeFourBit);

    uint8_t LCDDisplayOn = 0x0C;

    printk("LCDDisplayOn\n");

    sendCMD(dev, LCDDisplayOn);


    uint8_t LCDCursorTypeOn = 0X0E;    

    printk("LCDCursorTypeOn\n");

    sendCMD(dev, LCDCursorTypeOn);
        
    uint8_t LCDEntryModeThree = 0X06;    
    
    printk("LCDEntryModeThree\n");

    sendCMD(dev, LCDEntryModeThree);

    
    uint8_t LCDClearScreen = 0X01;    

   sendCMD(dev, LCDClearScreen);

}

void pcf857_Clear(const struct device * dev){
    int error;
    uint8_t LCDClearScreen = 0X01;

    printk("LCDLineAddressTwo\n");

   sendCMD(dev, LCDClearScreen);

}

void pcf857_LCDGOTO(const struct device * dev){
    int error;
    uint8_t LCDLineAddressOne= 0x80 | 0;

    printk("LCDLineAddressOne\n");
   
   sendCMD(dev, LCDLineAddressOne);

}
