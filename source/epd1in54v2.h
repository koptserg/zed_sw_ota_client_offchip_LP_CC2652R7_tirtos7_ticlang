#ifndef EPD1IN54V2_H
#define EPD1IN54V2_H

#include "zcl.h"

// Define names for GPIO pin indexes
#define CONFIG_GPIO_BUTTON_BUSY_EPD_INPUT 1 // DIO1
#define CONFIG_LED_CS_EPD_GPIO 11 // DIO11
#define CONFIG_LED_DC_EPD_GPIO 12 // DIO12
#define CONFIG_LED_RST_EPD_GPIO 0 // DIO0

#define EPD1IN54V2_APP_EPD_PARTIAL_EVT       0x1000
#define EPD1IN54V2_APP_EPD_DELAY_EVT       0x2000

// Display resolution
#define EPD_WIDTH       200
#define EPD_HEIGHT      200

// EPD1IN54 commands
#define DRIVER_OUTPUT_CONTROL                       0x01
#define GATE_VOLTAGE_CONTROL                        0x03
#define SOURCE_VOLTAGE_CONTROL                      0x04
#define BOOSTER_SOFT_START_CONTROL                  0x0C
#define GATE_SCAN_START_POSITION                    0x0F
#define DEEP_SLEEP_MODE                             0x10
#define DATA_ENTRY_MODE_SETTING                     0x11
#define SW_RESET                                    0x12
#define TEMPERATURE_SENSOR_CONTROL                  0x1A
#define BUILTINTEMPERATURE_SENSOR_CONTROL           0x18
#define MASTER_ACTIVATION                           0x20
#define DISPLAY_UPDATE_CONTROL_1                    0x21
#define DISPLAY_UPDATE_CONTROL_2                    0x22
#define WRITE_RAM                                   0x24
#define WRITE_RAM2                                  0x26
#define WRITE_VCOM_REGISTER                         0x2C
#define WRITE_LUT_REGISTER                          0x32
#define PROGRAM_OTP_SELECTION                       0x36
#define OTP_SELECTION_CONTROL_1                     0x37
#define OTP_SELECTION_CONTROL_2                     0x38
#define SET_DUMMY_LINE_PERIOD                       0x3A
#define SET_GATE_TIME                               0x3B
#define BORDER_WAVEFORM_CONTROL                     0x3C
#define OPTION_LUT_END                              0x3F
#define SET_RAM_X_ADDRESS_START_END_POSITION        0x44
#define SET_RAM_Y_ADDRESS_START_END_POSITION        0x45
#define SET_RAM_X_ADDRESS_COUNTER                   0x4E
#define SET_RAM_Y_ADDRESS_COUNTER                   0x4F
#define TERMINATE_FRAME_READ_WRITE                  0xFF

extern    unsigned long epd_width;
extern    unsigned long epd_height;

#endif /* EPD1IN54V2_H */

/* END OF FILE */
