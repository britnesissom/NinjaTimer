#ifndef HOMEAUTOMATIONRGBLED_H
#define HOMEAUTOMATIONRGBLED_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "Board.h"

/*********************************************************************
 * CONSTANTS
 */

typedef struct color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color;

#define SERVICE_ID_RGBLED    0x1140

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

#if Board_RGB_NUM_LEDS > 0
/*
 * Initialize RGB LED module
 */
extern void RGBLED_init(void);

/*
 * Task Event Processor for characteristic changes
 */
void RGBLED_processCharChangeEvt(uint8_t paramID);

/*
 * Reset RGB LED module
 */
extern void RGBLED_reset(void);

/*
 * Set specific LED color
 */
extern void RGBLED_SetLedColor(uint8_t r, uint8_t g, uint8_t b, bool isTime);

/*
 * Update LED colors
 */
void RGBLED_Update(void);

extern void RGBLED_UpdateTimeDigits(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5);

extern void RGBLED_UpdateScoreDigits(uint8_t digit1, uint8_t digit2);

void RGBLED_UpdateDigits(int startIndex, uint8_t number, uint8_t dp, bool isTime);

#else

/* RGB LED module not included */
//#define HomeAutomationRGBLED_init()
//#define HomeAutomationRGBLED_processCharChangeEvt(paramID)
//#define HomeAutomationRGBLED_processEvent()
//#define HomeAutomationRGBLED_reset()
//#define RGBLED_SetLedColor(index, r, g, b)
//#define RGBLED_Update()

#endif // Board_RGB_NUM_LEDS > 0

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HOMEAUTOMATIONRGBLED_H */
