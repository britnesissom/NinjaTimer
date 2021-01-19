/*********************************************************************
 * INCLUDES
 */
#include <uartlog/UartLog.h>
#include <ti/sysbios/knl/Task.h>

#include "osal_snv.h"
#include "gatt.h"
#include "gattservapp.h"
#include "util_rgb_led.h"
#include "Board.h"
#include "peripheral.h"
#include "util.h"
#include "rgbledservice.h"
#include "ws2812.h"

#if Board_RGB_NUM_LEDS > 0

/*********************************************************************
 * MACROS
 */

#define LED_IN_SEG 4
#define LED_IN_SCORE 3
#define NUM_LEDS (5 * (7 * LED_IN_SEG) + 3)
#define NUM_SCORE_LEDS (2 * (7 * LED_IN_SCORE))

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * CONSTANTS
 */
// Events
#define RGB_LED_COLOR_CHANGED_EVT            (1 << 0)

const color BLUE = { 150, 0, 218 };
const color ORANGE = { 73, 232, 0 };
const color BLACK = { 0, 0, 0 };

/*********************************************************************
 * GLOBAL VARIABLES
 */
color LEDs[Board_RGB_NUM_LEDS];

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

extern void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

/*********************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void RGBLED_StateChangeCB(uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      RGBLED_init
 *
 * @brief   Initialization function for the RGB LEDs
 *
 * @param   none
 *
 * @return  none
 */
void RGBLED_init(void)
{
  uint16_t i;

  // Add RGB LED service
  RGBLED_AddService();
  RGBLED_Setup(RGBLED_StateChangeCB);

#if Board_WS2812_NUM_LEDS > 0
  WS2812_init(Board_WS2812_SPI);
#endif

  // Turn off all LEDs
  for (i = 0; i < Board_RGB_NUM_LEDS; i++)
    RGBLED_SetLedColor(i, 0, 0, 0);
  RGBLED_Update();

  // Initialize the module state variables
  RGBLED_reset();
}

/*********************************************************************
 * @fn      RGBLED_processCharChangeEvt
 *
 * @brief   RGB LED event handling
 *
 * @param   event - event identifier
 *
 */
void RGBLED_processCharChangeEvt(uint8_t event)
{
  if (event == RGB_LED_COLOR_CHANGED_EVT)
  {
    uint8_t i; // colors[3 * Board_RGB_NUM_LEDS];

    RGBLED_GetParameter(RGB_LED_PARAM_COLOR, &LEDs);
    /* Set LED colors */
    for (i = 0; i < Board_RGB_NUM_LEDS; i++)
      RGBLED_SetLedColor(i, 0, 0, 0);
    /* Update */
    RGBLED_Update();
  }
}

/*********************************************************************
 * @fn      RGBLED_reset
 *
 * @brief   Reset RGB LED
 *
 * @param   none
 *
 * @return  none
 */
void RGBLED_reset(void)
{
  uint16_t i;

  // Set all LEDs to internal state
  for (i = 0; i < Board_RGB_NUM_LEDS; i++)
    RGBLED_SetLedColor(i, 0, 0, 0);

  RGBLED_Update();
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_StateChangeCB
 *
 * @brief   Callback function to be called when RGB LEDs are changed.
 *
 * @return  None.
 */
static void RGBLED_StateChangeCB(uint16_t connHandle, uint16_t svcUuid,
                                               uint8_t paramID, uint8_t *pValue, uint16_t len)
{
  // Wake up the application thread
  // See the service header file to compare paramID with characteristic.
  Log_info1("(CB) LED characteristic value change: svc(0x%04x). Sending msg to app.", (IArg)svcUuid);
  user_service_ValueChangeCB(connHandle, svcUuid, paramID, pValue, len);
}

/*********************************************************************
 * @fn      RGBLED_SetLedColor
 *
 * @brief   Set a specific LED's color.
 *
 * @param   on - the state to set the relay.
 * @param   r - the red portion of the color.
 * @param   g - the green portion of the color.
 * @param   b - the blue portion of the color.
 *
 * @return  None.
 */
void RGBLED_SetLedColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
  LEDs[index].r = r;
  LEDs[index].g = g;
  LEDs[index].b = b;
}

/*********************************************************************
 * @fn      RGBLED_Update
 *
 * @brief   Send color configuration to LEDs.
 *
 * @return  None.
 */
void RGBLED_Update(void)
{

#if Board_WS2812_NUM_LEDS > 0
  uint16_t i = 0, handled = 0;
  uint32_t sleep = 500 * (1000 / Clock_tickPeriod);

  for (; i - handled < Board_WS2812_NUM_LEDS; i++) {
      WS2812_setLEDcolor(i - handled, BLUE.r, BLUE.g, BLUE.b, WS2812_REFRESH);
      Task_sleep(sleep);
      WS2812_setLEDcolor(i - handled, 0, 0, 0, WS2812_REFRESH);
      Task_sleep(sleep);
  }
//  WS2812_refreshLEDs();
  handled += Board_WS2812_NUM_LEDS;
#endif

  RGBLED_SetParameter(RGB_LED_PARAM_COLOR, sizeof(LEDs), LEDs);
}

void RGBLED_UpdateTimeDigits(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5) {
    RGBLED_UpdateDigits(0, digit1, 0, true); // min 1
    RGBLED_UpdateDigits(7 * LED_IN_SEG, digit2, 0, true); // min 2
    RGBLED_UpdateDigits((14 * LED_IN_SEG), digit3, 2, true); // sec 1
    RGBLED_UpdateDigits((21 * LED_IN_SEG), digit4, 2, true); // sec 2
    RGBLED_UpdateDigits((28 * LED_IN_SEG), digit5, 3, true); // 1/10 sec
}

void RGBLED_UpdateScoreDigits(uint8_t digit1, uint8_t digit2) {
//    RGBLED_UpdateDigits((35 * LED_IN_SEG), digit1, 5, false);
//    RGBLED_UpdateDigits((35 * LED_IN_SEG) + (7 * LED_IN_SCORE), digit2, 5, false);

    if (digit2 == 0) {
        for (uint8_t i = 0; i < Board_WS2812_NUM_LEDS; i++) {
            WS2812_setLEDcolor(i, 0, 0, 0, WS2812_NOREFRESH);
        }
    }

    // testing with launchpad
    if (digit2 > 0 && digit2 <= Board_WS2812_NUM_LEDS) {
        WS2812_setLEDcolor(digit2 - 1, BLUE.r, BLUE.g, BLUE.b, WS2812_NOREFRESH);
    }

    WS2812_refreshLEDs();
}

/*********************************************************************
 * @fn      RGBLED_TimeUpdate
 *
 * @brief   Update LEDs based on current stopwatch time.
 *
 * @return  None.
 */
void RGBLED_UpdateDigits(int startIndex, uint8_t number, uint8_t dp, bool isTime)
{

#if Board_WS2812_NUM_LEDS > 0
    //  #    each 1 corresponds to a lit segment in the digit
    //  #        A 1
    //  #      ----------
    //  #     |          |
    //  #     |          |
    //  # F 6 |          | B 2
    //  #     |          |
    //  #     |     G 7  |
    //  #      ----------
    //  #     |          |
    //  #     |          |
    //  # E 5 |          | C 3
    //  #     |          |
    //  #     |     D 4  |    dp 8
    //  #      ----------

    uint16_t numbers[] = {
      0b00111111, // 0
      0b00000110, // 1
      0b01011011, // 2
      0b01001111, // 3
      0b01100110, // 4
      0b01101101, // 5
      0b01111101, // 6
      0b00000111, // 7
      0b01111111, // 8
      0b01101111, // 9
    };

    uint16_t leds_in_seg = (!isTime) ? LED_IN_SCORE : LED_IN_SEG;
    // color for score and time, respectively
    color scoreTimeColor = (!isTime) ? BLUE : ORANGE;

    // set color via RGB
    uint16_t index;
    color ledColor;
    for(uint16_t i = 0; i < 7; i++) {
      for(uint16_t j = 0; j < leds_in_seg; j++) {
        index = leds_in_seg * (i + startIndex/leds_in_seg) + j + dp;
        ledColor = ((numbers[number] & 1 << i) == 1 << i) ? scoreTimeColor : BLACK;
        WS2812_setLEDcolor(index, ledColor.r, ledColor.g, ledColor.b, WS2812_NOREFRESH);
      }
    }

    WS2812_setLEDcolor(14 * LED_IN_SEG, ORANGE.r, ORANGE.g, ORANGE.b, WS2812_NOREFRESH);    // first dot after two digits
    WS2812_setLEDcolor((14 * LED_IN_SEG) + 1, ORANGE.r, ORANGE.g, ORANGE.b, WS2812_NOREFRESH);  // second dot after two digits
    WS2812_setLEDcolor((14 * LED_IN_SEG) + 2, ORANGE.r, ORANGE.g, ORANGE.b, WS2812_NOREFRESH);  // seconds dot after 4 digits/2 dots

    WS2812_refreshLEDs();
#endif

  RGBLED_SetParameter(RGB_LED_PARAM_COLOR, sizeof(LEDs), LEDs);
}

#endif // Board_RGB_NUM_LEDS > 0

/*********************************************************************
*********************************************************************/
