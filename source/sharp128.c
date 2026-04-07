
#ifdef SHARP128

#include <zcl_samplesw.h>

// Driver header file
#include <ti/drivers/GPIO.h>
// TI Drivers Configuration
#include "ti_drivers_config.h"

#include <ti/drivers/SPI.h>

#include <ti/grlib/grlib.h>
#include <ti/display/Display.h>
#include <ti/display/DisplaySharp.h>
#include <ti/display/DisplayExt.h>
#include <imagesharp.h>
#include <sharp128.h>
#include <ti/grlib/button.h>

Graphics_Context *pContext;

static void Sharp128_Test(void);

static void DelayUs(uint16_t microSecs);
static void DelayMs(uint16_t delaytime);

void Sharp128_Init(void) {
  Display_Params    params;
  Display_Params_init(&params);
  hDisplay = Display_open(Display_Type_GRLIB, &params);
  pContext = DisplayExt_getGraphicsContext(hDisplay);

  Sharp128_Test();
}

static void Sharp128_Test(void) {
    Display_clear(hDisplay);
    Display_printf(hDisplay, 3, 3, "Hello ZIGBEE %d.%d", 3, 14);
    DelayMs(1000);

    Graphics_setForegroundColor(pContext, ClrBlack);
    Graphics_setBackgroundColor(pContext, ClrWhite);
    Graphics_clearDisplay(pContext);
    Graphics_Rectangle rect = {
          .sXMin = 10,
          .sXMax = 118,
          .sYMin = 10,
          .sYMax = 118,
    };
    Graphics_drawRectangle(pContext, &rect);
    Graphics_fillCircle(pContext, 64, 64, 20);
    Graphics_setFont(pContext, &g_sFontFixed6x8);
    int8_t *string = (int8_t *)("GRAPHICS LIB");
    Graphics_drawString(pContext, string, AUTO_STRING_LENGTH, 25, 25, GRAPHICS_OPAQUE_TEXT);
    Graphics_flushBuffer(pContext);
    DelayMs(1000);

    Graphics_clearDisplay(pContext);
    Graphics_drawImage(pContext, &TI_Logo_107x100_1BPP_UNCOMP, 10, 14);
    Graphics_flushBuffer(pContext);
    DelayMs(1000);
/*
    Graphics_clearDisplay(&g_sContext);
    int8_t *text_b1 = (int8_t *)("1234");
    Graphics_Button button_1 = {
                                .xMin = 10,
                                .xMax = 30,
                                .yMin = 10,
                                .yMax = 100,
                                .borderWidth = 2,
                                .selected = 1,
                                .fillColor = ClrBlack,
                                .borderColor = ClrWhite,
                                .selectedColor = ClrBlack,
                                .textColor = ClrWhite,
                                .selectedTextColor = ClrWhite,
                                .textXPos = 5,
                                .textYPos = 5,
                                .text = text_b1,
                                .font = &g_sFontFixed6x8,
    };
    Graphics_drawButton(&g_sContext, &button_1);
    Graphics_flushBuffer(&g_sContext);
*/
    Graphics_setForegroundColor(pContext, ClrWhite);
    Graphics_setBackgroundColor(pContext, ClrBlack);
    Graphics_clearDisplay(pContext);
    Graphics_flushBuffer(pContext);
    Display_printf(hDisplay, 3, 3, "Hello ZIGBEE %d.%d", 3, 14);

}

static void DelayUs(uint16_t microSecs) {
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

static void DelayMs(uint16_t delaytime) {
  while(delaytime--)
  {
      DelayUs(1000);
  }
}
#endif //end SHARP128
