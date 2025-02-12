
#ifdef EPD1IN54V2

//#include <Application/source/zcl_samplesw.h>
#include <zcl_samplesw.h>
#if defined (OTA_CLIENT_INTEGRATED)
#include "zcl_ota.h"
#include "ota_client.h"
#endif

#include "utc_clock.h"
#include "zcl_port.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
//#include "zstackapi.h"

// Driver header file
#include <ti/drivers/GPIO.h>
// TI Drivers Configuration
#include "ti_drivers_config.h"

#include <ti/drivers/SPI.h>

//#include <Application/source/epd1in54v2.h>
//#include <Application/source/imagedata.h>
//#include <Application/source/epdpaint.h>
#include <epd1in54v2.h>
#include <imagedata.h>
#include <epdpaint.h>

static void epd1in54v2_initialization(void);
static uint16_t epd1in54v2_process_loop(void);

static    void EpdSpiInit(void);
static    void EpdSpiClose(void);
static    void EpdInitFull(void);
static    void EpdInitPartial(void);
static    void EpdReset(void);

static    void EpdSetFrameMemoryXY(const unsigned char* image_buffer,int x, int y, int image_width, int image_height);
static    void EpdSetFrameMemoryImageXY(const unsigned char* image_buffer, int x, int y, int image_width, int image_height, uint8 invert);

static    void EpdSetFrameMemory(const unsigned char* image_buffer);
static    void EpdSetFrameMemoryBase(const unsigned char* image_buffer, uint8_t invert);
static    void EpdClearFrameMemory(unsigned char color);
static    void EpdClearFrameMemoryF(unsigned char color);
static    void EpdDisplayFrame(void);
static    void EpdDisplayFramePartial(void);
static    void EpdSleep(void);

static    void epd1in54Refresh(void);

static void DelayMs(unsigned int delaytime);
static void DelayUs(uint16_t microSecs);

static void EpdSendCommand(unsigned char command);
static void EpdSendData(unsigned char data);
static void WaitUntilIdle(void);

static void EpdSetLut(const unsigned char *lut);
static void EpdSetLutFull(const unsigned char *lut);
static void EpdSetMemoryArea(int x_start, int y_start, int x_end, int y_end);
static void EpdSetMemoryPointer(int x, int y);

unsigned long epd_width = EPD_WIDTH;
unsigned long epd_height = EPD_HEIGHT;

const unsigned char lut_full_update[159];
const unsigned char lut_partial_update[159];

SPI_Handle  spi;

// Semaphore used to post events to the application thread
static Semaphore_Handle epdSemHandle;

/* App service ID used for messaging with stack service task */
static uint8_t  epdServiceTaskId;
/* App service task events, set by the stack service task when sending a message */
static uint32_t epdServiceTaskEvents;

static uint16_t epdEvents = 0;

static Semaphore_Struct semStruct;

// Passed in function pointers to the NV driver
static NVINTF_nvFuncts_t *pfnZdlNV = NULL;

void sampleApp_task_1(NVINTF_nvFuncts_t *pfnNV)
{
  // Save and register the function pointers to the NV drivers
  pfnZdlNV = pfnNV;
  zclport_registerNV(pfnZdlNV, ZCL_PORT_SCENE_TABLE_NV_ID);

  // Initialize application
  epd1in54v2_initialization();

  // No return from task process
  epd1in54v2_process_loop();
}

void epd1in54v2_initialization(void)
{
    //create semaphores for messages / otaClientEvents
    Semaphore_Params epdSemParam;
    Semaphore_Params_init(&epdSemParam);
    epdSemParam.mode = ti_sysbios_knl_Semaphore_Mode_COUNTING;
    Semaphore_construct(&semStruct, 0, &epdSemParam);
    epdSemHandle = Semaphore_handle(&semStruct);

    epdServiceTaskId = OsalPort_registerTask(Task_self(), epdSemHandle, &epdServiceTaskEvents);

    // init SPI
    SPI_init();
    EpdSpiInit();
    //init EPD
    uint8_t zclApp_color = 0xFF;
    EpdInitFull();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFrame();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFrame();
    EpdInitPartial();
    epd1in54Refresh();

#ifndef CUI_DISABLE
    OsalPortTimers_startReloadTimer(epdServiceTaskId,  EPD1IN54V2_APP_EPD_DELAY_EVT, 1000);
#else
    OsalPortTimers_startReloadTimer(epdServiceTaskId,  EPD1IN54V2_APP_EPD_DELAY_EVT, 60000);
#endif
}

uint16_t epd1in54v2_process_loop(void)
{
    /* Forever loop */
  for(;;)
  {
            /* Wait for response message */
      if(Semaphore_pend(epdSemHandle, BIOS_WAIT_FOREVER))
      {
            if ( epdServiceTaskEvents & EPD1IN54V2_APP_EPD_DELAY_EVT )
            {
                EpdSpiInit();
                epd1in54Refresh();

                epdServiceTaskEvents &= ~EPD1IN54V2_APP_EPD_DELAY_EVT;
            }
            if ( epdServiceTaskEvents & EPD1IN54V2_APP_EPD_PARTIAL_EVT )
            {
                  if (GPIO_read(CONFIG_GPIO_BUTTON_BUSY_EPD_INPUT) != 1) {
                      OsalPortTimers_stopTimer(epdServiceTaskId,  EPD1IN54V2_APP_EPD_PARTIAL_EVT);
                      EpdSleep();
                      EpdSpiClose();
                  } else {
                      OsalPortTimers_startTimer(epdServiceTaskId,  EPD1IN54V2_APP_EPD_PARTIAL_EVT, 100);
                  }
                  epdServiceTaskEvents &= ~EPD1IN54V2_APP_EPD_PARTIAL_EVT;
            }
      }
  }
}

void EpdSpiInit(void) {
    GPIO_init();
    // Configure a button pin as input and configure its interrupt
    // Passing INT_ENABLE means you do not need to also call GPIO_enableInt()
    GPIO_setConfig(CONFIG_GPIO_BUTTON_BUSY_EPD_INPUT, GPIO_CFG_IN_NOPULL);
//    GPIO_setConfig(CONFIG_GPIO_BUTTON_BUSY_EPD_INPUT, GPIO_CFG_IN_PU);
    // Configure an LED output pin
    GPIO_setConfig(CONFIG_LED_CS_EPD_GPIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    GPIO_setConfig(CONFIG_LED_DC_EPD_GPIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    GPIO_setConfig(CONFIG_LED_RST_EPD_GPIO, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);


//    SPI_init();
    SPI_Params  spiParams;
    SPI_Params_init(&spiParams);
    spiParams.transferMode = SPI_MODE_BLOCKING;
//    spi = SPI_open(CONFIG_SPI_0, &spiParams);
    spi = SPI_open(CONFIG_SPI_1, &spiParams);
    if (spi == NULL) {
        // Error opening SPI
        while(1);
    }
}

void EpdSpiClose(void) {
    SPI_close(spi);
}

void EpdInitFull(void) {  
  EpdReset();
  WaitUntilIdle();
//  EpdSendCommand(SW_RESET);  //SWRESET
//  WaitUntilIdle();
  EpdSendCommand(DRIVER_OUTPUT_CONTROL);

  EpdSendData(0x27);
  EpdSendData(0x01);
  EpdSendData(0x00);
  
  EpdSendCommand(DATA_ENTRY_MODE_SETTING);
  EpdSendData(0x03);
  
  EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
  EpdSendCommand(BORDER_WAVEFORM_CONTROL);
  EpdSendData(0x01);
  EpdSendCommand(BUILTINTEMPERATURE_SENSOR_CONTROL);
  EpdSendData(0x80);

  EpdSetMemoryPointer(0, 0);
  WaitUntilIdle();
  EpdSetLutFull(lut_full_update);
  PaintPaint(image, 0, 0);
}

void EpdInitPartial(void) { 
    EpdReset();
    //WaitUntilIdle();
    EpdSetLut(lut_partial_update);
    EpdSendCommand(OTP_SELECTION_CONTROL_1); 
    EpdSendData(0x00);  
    EpdSendData(0x00);  
    EpdSendData(0x00);  
    EpdSendData(0x00); 
    EpdSendData(0x00);   
    EpdSendData(0x40);  
    EpdSendData(0x00);  
    EpdSendData(0x00);   
    EpdSendData(0x00);  
    EpdSendData(0x00);
    EpdSendCommand(BORDER_WAVEFORM_CONTROL);
    EpdSendData(0x80); 
    EpdSendCommand(DISPLAY_UPDATE_CONTROL_2); 
    EpdSendData(0xC0);   
    EpdSendCommand(MASTER_ACTIVATION); 
    WaitUntilIdle();
    PaintPaint(image, 0, 0);
}
/* SPI config pin CONFIG_SPI_0 - Any(SSI0)
CONFIG_PIN_SPI_SCLK - Any(DIO10/7 (LaunchPad SPI Bus))
CONFIG_PIN_SPI_MISO - Any(DIO8/14 (LaunchPad SPI Bus))
CONFIG_PIN_SPI_MOSI - Any(DIO9/15 (LaunchPad SPI Bus))
*/
/* SPI config pin CONFIG_SPI_1 - Any(SSI1)
CONFIG_GPIO_SPI_1_SCLK - Any(DIO30/28 (Header))
CONFIG_GPIO_SPI_1_POCI - Any(DIO29/27 (Header)) MISO
CONFIG_GPIO_SPI_1_PICO - Any(DIO28/26 (Header)) MOSI
*/
/* Control config pin
CONFIG_LED_CS_EPD - DIO11/18 (LaunchPad SPI Bus Chip Select)
CONFIG_LED_DC_EPD - DIO12/19 (Header)
CONFIG_LED_RST_EPD - DIO0/29 (Header)
CONFIG_BUTTON_BUSY_EPD - Any(DIO1/30 (Header))
 */
void EpdReset(void) {
    GPIO_write (CONFIG_LED_RST_EPD_GPIO, CONFIG_GPIO_LED_OFF);
    DelayMs(15); // 15 ms
    GPIO_write (CONFIG_LED_RST_EPD_GPIO, CONFIG_GPIO_LED_ON);
    DelayUs(15); // 15 us
}

static void EpdSendCommand(unsigned char command) {
    GPIO_write (CONFIG_LED_CS_EPD_GPIO, CONFIG_GPIO_LED_OFF); // CS = 0
    GPIO_write (CONFIG_LED_DC_EPD_GPIO, CONFIG_GPIO_LED_OFF); //command

    SPI_Transaction spiTransaction;
    uint8_t transmitBuffer[1];
    uint8_t receiveBuffer[1];
    bool transferOK;
    transmitBuffer[0] = command;

    spiTransaction.count = 1;
    spiTransaction.txBuf = transmitBuffer;
    spiTransaction.rxBuf = receiveBuffer;

    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK) {
        // Error in SPI or transfer already in progress.
    }
    GPIO_write (CONFIG_LED_DC_EPD_GPIO, CONFIG_GPIO_LED_ON); // data
    GPIO_write (CONFIG_LED_CS_EPD_GPIO, CONFIG_GPIO_LED_ON);  // CS = 1
}

static void EpdSendData(unsigned char data) {
//    HalLcd_HW_Write(data);
    GPIO_write (CONFIG_LED_CS_EPD_GPIO, CONFIG_GPIO_LED_OFF); // CS = 0
    GPIO_write (CONFIG_LED_DC_EPD_GPIO, CONFIG_GPIO_LED_ON); // data

    SPI_Transaction spiTransaction;
    uint8_t transmitBuffer[1];
    uint8_t receiveBuffer[1];
    bool transferOK;
    transmitBuffer[0] = data;

    spiTransaction.count = 1;
    spiTransaction.txBuf = transmitBuffer;
    spiTransaction.rxBuf = receiveBuffer;

    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK) {
        // Error in SPI or transfer already in progress.
    }

    GPIO_write (CONFIG_LED_CS_EPD_GPIO, CONFIG_GPIO_LED_ON);  // CS = 1
}

static void WaitUntilIdle(void) {
    uint8_t error_time = 200; // over 2.5 sec return
//     while(HAL_LCD_BUSY == 1) {      //LOW: idle, HIGH: busy
    while(GPIO_read(CONFIG_GPIO_BUTTON_BUSY_EPD_INPUT) == 1) {      //LOW: idle, HIGH: busy
        DelayMs(10);
        error_time = error_time - 1;
        if (error_time == 0){    
          EpdReset();
          return;
        }
    } 
}

void DelayUs(uint16_t microSecs) {
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

void DelayMs(unsigned int delaytime) {
  while(delaytime--)
  {
      DelayUs(1000);
  }
}

const unsigned char lut_full_update[159] =
{																						
0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x8,	0x1,	0x0,	0x8,	0x1,	0x0,	0x2,					
0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x22,	0x22,	0x22,	0x22,	0x22,	0x22,	0x0,	0x0,	0x0,			
0x22,	0x17,	0x41,	0x0,	0x32,	0x20
};

const unsigned char lut_partial_update[159] = {
0x0,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x80,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x40,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0xF,0x0,0x0,0x0,0x0,0x0,0x0,
0x1,0x1,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x22,0x22,0x22,0x22,0x22,0x22,0x0,0x0,0x0,
0x02,0x17,0x41,0xB0,0x32,0x28,
};


static void EpdSetLutFull(const unsigned char *lut) {
    EpdSetLut((const unsigned char *)lut);
	EpdSendCommand(OPTION_LUT_END);
	EpdSendData(*(lut+153));
	EpdSendCommand(GATE_VOLTAGE_CONTROL);
	EpdSendData(*(lut+154));
	EpdSendCommand(SOURCE_VOLTAGE_CONTROL);
	EpdSendData(*(lut+155));	// VSH
	EpdSendData(*(lut+156));	// VSH2
	EpdSendData(*(lut+157));	// VSL
	EpdSendCommand(WRITE_VCOM_REGISTER);
	EpdSendData(*(lut+158));
}  


static void EpdSetLut(const unsigned char *lut) {
  unsigned char count;
  EpdSendCommand(WRITE_LUT_REGISTER);
  for(count=0; count<153; count++) 
    EpdSendData(lut[count]); 
  WaitUntilIdle();
}

void EpdSetFrameMemoryImageXY(const unsigned char* image_buffer, int x, int y, int image_width, int image_height, uint8 invert) {
    int x_end;
    int y_end;

    if (
        image_buffer == NULL ||
        x < 0 || image_width < 0 ||
        y < 0 || image_height < 0
    ) {
        return;
    }
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    x &= 0xF8;
    image_width &= 0xF8;
    if (x + image_width >= epd_width) {
        x_end = epd_width - 1;
    } else {
        x_end = x + image_width - 1;
    }
    if (y + image_height >= epd_height) {
        y_end = epd_height - 1;
    } else {
        y_end = y + image_height - 1;
    }
    
    EpdSetMemoryArea(x, y, x_end, y_end);
    EpdSetMemoryPointer(x, y);
    EpdSendCommand(WRITE_RAM);
    /* send the image data */
    for (int j = 0; j < y_end - y + 1; j++) {
        for (int i = 0; i < (x_end - x + 1) / 8; i++) {
          uint8_t inv_image = image_buffer[i + j * (image_width / 8)];
          if (invert){
            inv_image ^= 0x00;  
          } else {  
            inv_image ^= 0xFF;
          }
          EpdSendData((uint8_t)( inv_image ));
//            EpdSendData(image_buffer[i + j * (image_width / 8)]);
        }
    }
}

void EpdSetFrameMemoryXY(const unsigned char* image_buffer, int x, int y, int image_width, int image_height) {
    int x_end;
    int y_end;

    if (
        image_buffer == NULL ||
        x < 0 || image_width < 0 ||
        y < 0 || image_height < 0
    ) {
        return;
    }
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    x &= 0xF8;
    image_width &= 0xF8;
    if (x + image_width >= epd_width) {
        x_end = epd_width - 1;
    } else {
        x_end = x + image_width - 1;
    }
    if (y + image_height >= epd_height) {
        y_end = epd_height - 1;
    } else {
        y_end = y + image_height - 1;
    }
    
    EpdSetMemoryArea(x, y, x_end, y_end);
    EpdSetMemoryPointer(x, y);
    EpdSendCommand(WRITE_RAM);
    /* send the image data */
    for (int j = 0; j < y_end - y + 1; j++) {
        for (int i = 0; i < (x_end - x + 1) / 8; i++) {
            EpdSendData(image_buffer[i + j * (image_width / 8)]);
        }
    }
}

void EpdSetFrameMemory(const unsigned char* image_buffer) {
    EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
    EpdSetMemoryPointer(0, 0);
    EpdSendCommand(WRITE_RAM);
    /* send the image data */
    for (int i = 0; i < epd_width / 8 * epd_height; i++) {
        EpdSendData((uint8_t)(image_buffer[i]));
    }
}

void EpdSetFrameMemoryBase(const unsigned char* image_buffer, uint8_t invert) {
  EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
  EpdSetMemoryPointer(0, 0);
  EpdSendCommand(WRITE_RAM);
  /* send the image data */
  for (int i = 0; i < epd_width / 8 * epd_height; i++) {
    uint8_t inv_image = image_buffer[i];
    if (invert){
      inv_image ^= 0x00;  
    } else {  
      inv_image ^= 0xFF;
    }
    EpdSendData((uint8_t)( inv_image ));
  }
//  EpdSendCommand(WRITE_RAM2);
  /* send the image data */
//  for (int i = 0; i < epd_width / 8 * epd_height; i++) {
//    uint8_t inv_image = image_buffer[i];
//    if (invert){
//      inv_image ^= 0x00;  
//    } else {  
//      inv_image ^= 0xFF;
//    }
//    EpdSendData((uint8_t)( inv_image ));
//  }
}


void EpdClearFrameMemory(unsigned char color) {
    EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
    EpdSetMemoryPointer(0, 0);
    EpdSendCommand(WRITE_RAM);
    /* send the color data */
    for (int i = 0; i < epd_width / 8 * epd_height; i++) {
        EpdSendData(color);
    }
    EpdSendCommand(WRITE_RAM2);
    /* send the color data */
    for (int i = 0; i < epd_width / 8 * epd_height; i++) {
        EpdSendData(color);
    }
}

void EpdClearFrameMemoryF(unsigned char color) {
    EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
    EpdSetMemoryPointer(0, 0);
    
    EpdSendCommand(WRITE_RAM2);
    /* send the color data */
    for (int i = 0; i < epd_width / 8 * epd_height; i++) {
        EpdSendData(0x00);
    }
    
    EpdSendCommand(WRITE_RAM);
    /* send the color data */
    for (int i = 0; i < epd_width / 8 * epd_height; i++) {
        EpdSendData(color);
    }
    
}


static void EpdSetMemoryArea(int x_start, int y_start, int x_end, int y_end) {
    EpdSendCommand(SET_RAM_X_ADDRESS_START_END_POSITION);
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    EpdSendData((x_start >> 3) & 0xFF);
    EpdSendData((x_end >> 3) & 0xFF);
    EpdSendCommand(SET_RAM_Y_ADDRESS_START_END_POSITION);
    EpdSendData(y_start & 0xFF);
    EpdSendData((y_start >> 8) & 0xFF);
    EpdSendData(y_end & 0xFF);
    EpdSendData((y_end >> 8) & 0xFF);
}


static void EpdSetMemoryPointer(int x, int y) {
    EpdSendCommand(SET_RAM_X_ADDRESS_COUNTER);
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    EpdSendData((x >> 3) & 0xFF);
    EpdSendCommand(SET_RAM_Y_ADDRESS_COUNTER);
    EpdSendData(y & 0xFF);
    EpdSendData((y >> 8) & 0xFF);
}


void EpdDisplayFramePartial(void) {
  EpdSendCommand(DISPLAY_UPDATE_CONTROL_2);
  EpdSendData(0xFF);
  EpdSendCommand(MASTER_ACTIVATION);
//  WaitUntilIdle();
}


void EpdDisplayFrame(void) {
  EpdSendCommand(DISPLAY_UPDATE_CONTROL_2);
  EpdSendData(0xC7);
  EpdSendCommand(MASTER_ACTIVATION);
  WaitUntilIdle();
}


void EpdSleep(void) {
    EpdSendCommand(DEEP_SLEEP_MODE);
    EpdSendData(0x01);
}

void epd1in54Refresh(void)
{
    EpdReset(); //disable sleep EPD
    PaintSetInvert(1);

      //status network
      // landscape
      if ( bdbAttributes.bdbNodeIsOnANetwork ){
        EpdSetFrameMemoryImageXY(IMAGE_ONNETWORK, 184, 0, 16, 16, 1);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_OFFNETWORK, 184, 0, 16, 16, 1);
      }

      //percentage
      char perc_string[] = {' ', ' ', ' ', ' ', '\0'};
      if (zclSampleSw_PercentageRemainig != 0xFF) {
        perc_string[0] = zclSampleSw_PercentageRemainig/2 / 100 % 10 + '0';
        perc_string[1] = zclSampleSw_PercentageRemainig/2 / 10 % 10 + '0';
        perc_string[2] = zclSampleSw_PercentageRemainig/2 % 10 + '0';
        perc_string[3] = '%';
      }
      PaintSetWidth(16);
      PaintSetHeight(48);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, perc_string, &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 144, PaintGetWidth(), PaintGetHeight());
      if (zclSampleSw_PercentageRemainig != 0xFF) {
        if(zclSampleSw_PercentageRemainig/2 > 75){
          EpdSetFrameMemoryImageXY(IMAGE_BATTERY_100, 184, 116, 16, 25, 1);
        } else if (zclSampleSw_PercentageRemainig/2 <= 75 && zclSampleSw_PercentageRemainig/2 > 50) {
          EpdSetFrameMemoryImageXY(IMAGE_BATTERY_75, 184, 116, 16, 25, 1);
        } else if (zclSampleSw_PercentageRemainig/2 <= 50 && zclSampleSw_PercentageRemainig/2 > 25) {
          EpdSetFrameMemoryImageXY(IMAGE_BATTERY_50, 184, 116, 16, 25, 1);
        } else if (zclSampleSw_PercentageRemainig/2 <= 25 && zclSampleSw_PercentageRemainig/2 > 6) {
          EpdSetFrameMemoryImageXY(IMAGE_BATTERY_25, 184, 116, 16, 25, 1);
        } else if (zclSampleSw_PercentageRemainig/2 <= 6 && zclSampleSw_PercentageRemainig/2 > 0) {
          EpdSetFrameMemoryImageXY(IMAGE_BATTERY_0, 184, 116, 16, 25, 1);
        }
      }

    // clock init Firmware build date 20/08/2021 13:47
    // Update RTC and get new clock values
      UTCTimeStruct time;
      UTC_convertUTCTime(&time, UTC_getClock());

      char time_string[] = {'0', '0', ':', '0', '0', ':', '0', '0','\0'};
      time_string[0] = time.hour / 10 % 10 + '0';
      time_string[1] = time.hour % 10 + '0';
      time_string[3] = time.minutes / 10 % 10 + '0';
      time_string[4] = time.minutes % 10 + '0';
      time_string[6] = time.seconds / 10 % 10 + '0';
      time_string[7] = time.seconds % 10 + '0';

      // covert UTCTimeStruct date and month to display
      time.day = time.day + 1;
      time.month = time.month + 1;
      char date_string[] = {'0', '0', '/', '0', '0', '/', '0', '0', '\0'};
      date_string[0] = time.day /10 % 10  + '0';
      date_string[1] = time.day % 10 + '0';
      date_string[3] = time.month / 10 % 10 + '0';
      date_string[4] = time.month % 10 + '0';
      date_string[6] = time.year / 10 % 10 + '0';
      date_string[7] = time.year % 10 + '0';

      // landscape
        PaintSetWidth(48);
        PaintSetHeight(200);
        PaintSetRotate(ROTATE_90);
        PaintClear(UNCOLORED);
        PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
        EpdSetFrameMemoryXY(PaintGetImage(), 136, 4, PaintGetWidth(), PaintGetHeight());

      //landscape
        PaintSetWidth(16);
        PaintSetHeight(100);
        PaintSetRotate(ROTATE_90);
        PaintClear(UNCOLORED);
        PaintDrawStringAt(10, 0, date_string, &Font16, COLORED);
        EpdSetFrameMemoryXY(PaintGetImage(), 120, 100, PaintGetWidth(), PaintGetHeight());

  //    uint8 day_week = (uint16)floor((float)(zclSampleSw_GenTime_TimeUTC/86400)) % 7;
      uint8 day_week = (uint16)(float)(zclSampleSw_GenTime_TimeUTC/86400) % 7;
      char* day_string = "";
      if (day_week == 5) {
        day_string = "Thursday";
      } else if (day_week == 6) {
        day_string = " Friday ";
      } else if (day_week == 0) {
        day_string = "Saturday";
      } else if (day_week == 1) {
        day_string = " Sunday";
      } else if (day_week == 2) {
        day_string = " Monday";
      } else if (day_week == 3) {
        day_string = "Tuesday";
      } else if (day_week == 4) {
        day_string = "Wednesday";
      }

      //landscape
        PaintSetWidth(16);
        PaintSetHeight(100);
        PaintSetRotate(ROTATE_90);
        PaintClear(UNCOLORED);
        PaintDrawStringAt(0, 0, day_string, &Font16, COLORED);
        EpdSetFrameMemoryXY(PaintGetImage(), 120, 4, PaintGetWidth(), PaintGetHeight());

        //OTA status
        char ota_string[] = {'O','T','A', ':', ' ', ' ', ' ', ' ', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '\0'};
        uint32_t progress = (uint32_t)((100 * zclSampleSw_FileOffset) / zclOTA_DownloadedImageSize);
        ota_string[4] = progress / 100 % 10 + '0';
        ota_string[5] = progress / 10 % 10 + '0';
        ota_string[6] = progress % 10 + '0';

        if (progress >= 10) { ota_string[8] = '#'; }
        if (progress >= 20) { ota_string[9] = '#'; }
        if (progress >= 30) { ota_string[10] = '#'; }
        if (progress >= 40) { ota_string[11] = '#'; }
        if (progress >= 50) { ota_string[12] = '#'; }
        if (progress >= 60) { ota_string[13] = '#'; }
        if (progress >= 70) { ota_string[14] = '#'; }
        if (progress >= 80) { ota_string[15] = '#'; }
        if (progress >= 90) { ota_string[16] = '#'; }
        if (progress == 100) { ota_string[17] = '#'; }

        if (zclSampleSw_ImageUpgradeStatus == OTA_STATUS_IN_PROGRESS) {
        //landscape
          PaintSetWidth(16);
          PaintSetHeight(200);
          PaintSetRotate(ROTATE_90);
          PaintClear(UNCOLORED);
          PaintDrawStringAt(0, 0, ota_string, &Font16, COLORED);
          EpdSetFrameMemoryXY(PaintGetImage(), 104, 4, PaintGetWidth(), PaintGetHeight());
        }

    //Illuminance
    char illum_string[] = {'0','0', '0', '0', '0', '\0'};
  #ifdef BH1750
    illum_string[0] = zclSampleSw_IlluminanceMeasurment_MeasuredValue / 10000 % 10 + '0';
    illum_string[1] = zclSampleSw_IlluminanceMeasurment_MeasuredValue / 1000 % 10 + '0';
    illum_string[2] = zclSampleSw_IlluminanceMeasurment_MeasuredValue / 100 % 10 + '0';
    illum_string[3] = zclSampleSw_IlluminanceMeasurment_MeasuredValue / 10 % 10 + '0';
    illum_string[4] = zclSampleSw_IlluminanceMeasurment_MeasuredValue % 10 + '0';
  #endif
    //landscape
      PaintSetWidth(32);
      PaintSetHeight(80);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, illum_string, &Font32, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 64, 120, PaintGetWidth(), PaintGetHeight());
      PaintSetWidth(16);
      PaintSetHeight(33);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, "Lux", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 48, 120, PaintGetWidth(), PaintGetHeight());

    //temperature
    char temp_string[] = {'0', '0', '.', '0', '0', '\0'};
  #ifdef BME280
    temp_string[0] = zclSampleSw_Temperature_Sensor_MeasuredValue / 1000 % 10 + '0';
    temp_string[1] = zclSampleSw_Temperature_Sensor_MeasuredValue / 100 % 10 + '0';
    temp_string[3] = zclSampleSw_Temperature_Sensor_MeasuredValue / 10 % 10 + '0';
    temp_string[4] = zclSampleSw_Temperature_Sensor_MeasuredValue % 10 + '0';
  #endif
    //landscape
      PaintSetWidth(32);
      PaintSetHeight(80);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, temp_string, &Font32, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 64, 16, PaintGetWidth(), PaintGetHeight());
      PaintSetWidth(16);
      PaintSetHeight(22);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 48, 16, PaintGetWidth(), PaintGetHeight());

    //humidity
    char hum_string[] = {'0', '0', '.', '0', '0', '\0'};
  #ifdef BME280
    hum_string[0] = zclSampleSw_HumiditySensor_MeasuredValue / 1000 % 10 + '0';
    hum_string[1] = zclSampleSw_HumiditySensor_MeasuredValue / 100 % 10 + '0';
    hum_string[3] = zclSampleSw_HumiditySensor_MeasuredValue / 10 % 10 + '0';
    hum_string[4] = zclSampleSw_HumiditySensor_MeasuredValue % 10 + '0';
  #endif
    // landscape
      PaintSetWidth(32);
      PaintSetHeight(80);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, hum_string, &Font32, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 16, 16, PaintGetWidth(), PaintGetHeight());
      PaintSetWidth(16);
      PaintSetHeight(33);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, "%Ha", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 1, 16, PaintGetWidth(), PaintGetHeight());

    //pressure
    char pres_string[] = {'0', '0', '0', '0', '.', '0', '\0'};
  #ifdef BME280
    pres_string[0] = zclSampleSw_PressureSensor_MeasuredValue / 1000 % 10 + '0';
    pres_string[1] = zclSampleSw_PressureSensor_MeasuredValue / 100 % 10 + '0';
    pres_string[2] = zclSampleSw_PressureSensor_MeasuredValue / 10 % 10 + '0';
    pres_string[3] = zclSampleSw_PressureSensor_MeasuredValue % 10 + '0';
  #endif
    //landscape
      PaintSetWidth(32);
      PaintSetHeight(64);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, pres_string, &Font32, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 16, 120, PaintGetWidth(), PaintGetHeight());
      PaintSetWidth(16);
      PaintSetHeight(33);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, "hPa", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 1, 120, PaintGetWidth(), PaintGetHeight());

      EpdDisplayFramePartial();
  //    OsalPortTimers_startTimer(appServiceTaskId,  SAMPLEAPP_APP_EPD_PARTIAL_EVT, 100);
    OsalPortTimers_startTimer(epdServiceTaskId,  EPD1IN54V2_APP_EPD_PARTIAL_EVT, 100);

  //  EpdSleep();
}

#endif //end EPD1IN54V2
