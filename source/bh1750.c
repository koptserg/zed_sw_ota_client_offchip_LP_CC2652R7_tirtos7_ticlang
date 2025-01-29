
#ifdef BH1750
//#include <Application/source/bh1750.h>
#include <bh1750.h>
#include "ti_drivers_config.h"
// Import I2C Driver definitions
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

I2C_Handle i2cHandle;
static bool status = false;

uint8_t bh1750_MTreg = (uint8_t)BH1750_DEFAULT_MTREG;
uint8_t bh1750_mode = CONTINUOUS_HIGH_RES_MODE;

/**
 * Configure BH1750 with specified mode
 * @param mode Measurement mode
 */
bool bh1750_init(uint8_t mode) {
/*
    // initialize optional I2C bus parameters
    I2C_Params params;
//    I2CCC26XX_I2CPinCfg pinCfg;
    I2C_Params_init(&params);
//    params.bitRate = I2C_400kHz;
//    pinCfg.pinSDA = CONFIG_GPIO_I2C_0_SDA;
//    pinCfg.pinSCL = CONFIG_GPIO_I2C_0_SCL;
//    params.custom = &pinCfg;
    // Open I2C bus for usage
    i2cHandle_bh1750 = I2C_open(CONFIG_I2C_0, &params);
    if (i2cHandle_bh1750 == NULL) {
        // Error opening I2C
        return false;
    }
*/
  bh1750_Write(BH1750_RESET);

  bh1750_Write(BH1750_POWER_ON);

  bh1750_Write(ONE_TIME_LOW_RES_MODE);
  bh1750_WaitMs(10);
  uint16_t initread = 0xFFFF;
  initread = (uint16_t)bh1750_Read();

  if((uint16_t)initread != 0x0000){
    bh1750_mode = mode;
    bh1750_Write(mode);
    bh1750_setMTreg(bh1750_MTreg);

    bh1750_PowerDown();

    return true;
  }
  return false;
}

/**
 * Configure BH1750 MTreg value
 * MT reg = Measurement Time register
 * @param MTreg a value between 32 and 254. Default: 69
 * @return bool true if MTReg successful set
 *      false if MTreg not changed or parameter out of range
 */
bool bh1750_setMTreg(uint8_t MTreg) {
  bh1750_MTreg = MTreg;
  //Bug: lowest value seems to be 32!
  if (MTreg <= 31 || MTreg > 254) {
//    LREPMaster("[BH1750] ERROR: MTreg out of range\r\n");
    return 0;
  }
  // Send MTreg and the current mode to the sensor
  //   High bit: 01000_MT[7,6,5]
  //    Low bit: 011_MT[4,3,2,1,0]
  bh1750_Write((0x08 << 3) | (MTreg >> 5));
  bh1750_Write((0x03 << 5) | (MTreg & 0x1F));

  // Wait a few moments to wake up
  bh1750_WaitMs(10);

  return 0;
}

void bh1750_PowerDown(void) {
  if (bh1750_mode == CONTINUOUS_HIGH_RES_MODE || bh1750_mode == CONTINUOUS_HIGH_RES_MODE_2 || bh1750_mode == CONTINUOUS_LOW_RES_MODE) {
    bh1750_Write(BH1750_POWER_DOWN);
  }
}

float bh1750_Read(void) {
    I2C_Transaction i2cTransaction = {0};
    uint8_t readBuffer[2];

    i2cTransaction.targetAddress = BH1750_I2CADDR;
    i2cTransaction.writeBuf = NULL;
    i2cTransaction.writeCount = 0;
    i2cTransaction.readBuf = readBuffer;
    i2cTransaction.readCount = 2;

    status = I2C_transfer(i2cHandle, &i2cTransaction);

    if (status == false) {
        if (i2cTransaction.status == I2C_STATUS_ADDR_NACK) {
            // I2C target address not acknowledged
        }
    }

    uint16_t value16 = ((readBuffer[0] << 8) | readBuffer[1]);
    float level = 0;
    level = value16;
    if (bh1750_MTreg != BH1750_DEFAULT_MTREG) {
      level *= (float)((uint8_t)BH1750_DEFAULT_MTREG/(float)bh1750_MTreg);
    }
    if (bh1750_mode == ONE_TIME_HIGH_RES_MODE_2 || bh1750_mode == CONTINUOUS_HIGH_RES_MODE_2) {
      level /= 2;
    }
    // Convert raw value to lux
    level /= BH1750_CONV_FACTOR;
    return level;
}

void bh1750_Write(uint8_t mode) {
    I2C_Transaction i2cTransaction = {0};
    uint8_t writeBuffer[1];
    writeBuffer[0] = mode;

    i2cTransaction.targetAddress = BH1750_I2CADDR;
    i2cTransaction.writeBuf = writeBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;

    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if (status == false) {
        // Unsuccessful I2C transfer
        if (i2cTransaction.status == I2C_STATUS_ADDR_NACK) {
            // I2C target address not acknowledged
        }
    }

  switch (mode) {
    case CONTINUOUS_HIGH_RES_MODE:
    case CONTINUOUS_HIGH_RES_MODE_2:
    case CONTINUOUS_LOW_RES_MODE:
    case ONE_TIME_HIGH_RES_MODE:
    case ONE_TIME_HIGH_RES_MODE_2:
    case ONE_TIME_LOW_RES_MODE:
      bh1750_mode = mode;
      break;
    default:
      break;

  }
}

void bh1750_WaitUs(uint16_t microSecs) {
  while(microSecs--) {
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  }
}

void bh1750_WaitMs(unsigned int delaytime) {
  while(delaytime--)
  {
    bh1750_WaitUs(1000);
  }
}

#endif //BH1750
