#ifndef BH1750_H
#define BH1750_H

#include "zcl.h"

// No active state
#define BH1750_POWER_DOWN 0x00

// Waiting for measurement command
#define BH1750_POWER_ON 0x01

// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_RESET 0x07

// Default MTreg value
#define BH1750_DEFAULT_MTREG 69

#define UNCONFIGURED  0
       // Measurement at 1 lux resolution. Measurement time is approx 120ms.
#define CONTINUOUS_HIGH_RES_MODE 0x10
      // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
#define CONTINUOUS_HIGH_RES_MODE_2 0x11
      // Measurement at 4 lux resolution. Measurement time is approx 16ms.
#define CONTINUOUS_LOW_RES_MODE 0x13
      // Measurement at 1 lux resolution. Measurement time is approx 120ms.
#define ONE_TIME_HIGH_RES_MODE 0x20
      // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
#define ONE_TIME_HIGH_RES_MODE_2 0x21
      // Measurement at 4 lux resolution. Measurement time is approx 16ms.
#define ONE_TIME_LOW_RES_MODE 0x23

#define BH1750_CONV_FACTOR 1.2

#define BH1750_I2CADDR 0x23

//extern uint8 bh1750_mode = CONTINUOUS_HIGH_RES_MODE;
//extern uint8 bh1750_addr = BH1750_I2CADDR;
//extern uint8 bh1750_mode;
//extern uint8 bh1750_addr;

extern bool bh1750_init(uint8_t mode);
extern bool bh1750_setMTreg(uint8_t MTreg);
extern float bh1750_Read(void);
extern void bh1750_Write(uint8_t mode);
extern void bh1750_PowerDown(void);

void bh1750_WaitUs(uint16_t microSecs);
void bh1750_WaitMs(unsigned int delaytime);

#endif
