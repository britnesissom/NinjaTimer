/*********************************************************************
 * INCLUDES
 */
#include <uartlog/UartLog.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include <stdio.h>
#include <string.h>

#include "osal_snv.h"
#include "util.h"
#include "util_rgb_led.h"
#include "Board.h"
#include "rgbledservice.h"
#include "ws2812.h"

#if Board_RGB_NUM_LEDS > 0

/*********************************************************************
 * MACROS
 */

#define SNV_TIME_COLOR_ID       0x80
#define SNV_SCORE_COLOR_ID      0x81
#define SNV_BUF_LEN             12      // buf - rrr,ggg,bbb

#define TASK_STACK_SIZE         512

#define LED_IN_TIME_SEG         4
#define LED_IN_SCORE_SEG        3
#define NUM_TIME_LEDS           (5 * (7 * LED_IN_TIME_SEG) + 3)
#define NUM_SCORE_LEDS          (2 * (7 * LED_IN_SCORE_SEG))

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * CONSTANTS
 */
#define RGB_LED_COLOR_CHANGED_EVT            (1 << 0)
#define BLUE                                 { 150, 0, 218 }    // GRB
#define ORANGE                               { 73, 232, 0 }     // GRB

const color BLACK = { 0, 0, 0 };

/*********************************************************************
 * GLOBAL VARIABLES
 */
color LEDs[Board_RGB_NUM_LEDS];
char buf[SNV_BUF_LEN];

// Task configuration
Task_Struct ledTask;
Char ledTaskStack[TASK_STACK_SIZE];

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

static color timeColor = ORANGE;
static color scoreColor = BLUE;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void RGBLED_StateChangeCB(uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void RGBLED_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = ledTaskStack;
  taskParams.stackSize = TASK_STACK_SIZE;
  taskParams.priority = 1;

  Task_construct(&ledTask, RGBLED_taskFxn, &taskParams, NULL);
}

/*
 * @brief   Task that retrieves led color values from osal_snv
 *          and assigns them to local variables
 *
 * @param   a0, a1
 *
 * @return  None
 */
static void RGBLED_taskFxn(UArg a0, UArg a1)
{
    RGBLED_initLEDColor();
}

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
  // Add RGB LED service
  RGBLED_AddService();
  RGBLED_initLEDColor();

#if Board_WS2812_NUM_LEDS > 0
  WS2812_init(Board_WS2812_SPI);
#endif

  // reset LEDs to black
  RGBLED_reset();
}

void RGBLED_initLEDColor(void) {

    uint8_t status = SUCCESS;

    status = osal_snv_read(SNV_TIME_COLOR_ID, SNV_BUF_LEN, (char *)buf);

    // color hasn't been saved to snv yet
    if (status != SUCCESS) {
        // save time led color for first time
        System_snprintf(buf, SNV_BUF_LEN, "%d,%d,%d", timeColor.r, timeColor.g, timeColor.b);
        status = osal_snv_write(SNV_TIME_COLOR_ID, SNV_BUF_LEN, (char *)buf);
    } else {
        getRGBComponents(true);
    }

    // reset char array for score colors
    memset(buf, 0, sizeof buf);
    status = osal_snv_read(SNV_SCORE_COLOR_ID, SNV_BUF_LEN, (char *)buf);

    // color hasn't been saved to snv yet
    if (status != SUCCESS) {
        // save score led color for first time
        System_snprintf(buf, SNV_BUF_LEN, "%d,%d,%d", scoreColor.r, scoreColor.g, scoreColor.b);
        status = osal_snv_write(SNV_SCORE_COLOR_ID, SNV_BUF_LEN, (char *)buf);
    } else {
        getRGBComponents(false);
    }
}

void getRGBComponents(bool isTime) {
    char *token;
    char *savePtr;
    char *rgb[3];
    uint8_t i = 0;

    token = strtok_r((char *)buf, ",", &savePtr);

    /* walk through other tokens */
    while(token != NULL) {
        rgb[i] = token;
        i++;

        token = strtok_r(NULL, ",", &savePtr);
    }

    RGBLED_SetLedColor(atoi(rgb[0]), atoi(rgb[1]), atoi(rgb[2]), isTime);
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
    RGBLED_GetParameter(RGB_LED_PARAM_COLOR, &LEDs);
    /* Set LED colors */
    RGBLED_reset();
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
  // Set all LEDs to internal state
  for (uint8_t i = 0; i < Board_WS2812_NUM_LEDS; i++) {
      WS2812_setLEDcolor(i, 0, 0, 0, WS2812_NOREFRESH);
  }

  WS2812_refreshLEDs();
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_StateChangeCB
 *
 * @brief   Callback function to be called when RGB LEDs are changed.
 *
 * @return  None.
 */
/*static void RGBLED_StateChangeCB(uint16_t connHandle, uint16_t svcUuid,
                                               uint8_t paramID, uint8_t *pValue, uint16_t len)
{
  // Wake up the application thread
  // See the service header file to compare paramID with characteristic.
  Log_info1("(CB) LED characteristic value change: svc(0x%04x). Sending msg to app.", svcUuid);
  user_service_ValueChangeCB(connHandle, svcUuid, paramID, pValue, len);
}*/

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
void RGBLED_SetLedColor(uint8_t r, uint8_t g, uint8_t b, bool isTime)
{
    uint8_t status = SUCCESS;

    memset(buf, 0, sizeof buf);
    System_snprintf(buf, SNV_BUF_LEN, "%d,%d,%d", r, g, b);

    // colors are GRB so need to swap r and g values
    if (isTime) {
        timeColor.r = g;
        timeColor.g = r;
        timeColor.b = b;
        status = osal_snv_write(SNV_TIME_COLOR_ID, SNV_BUF_LEN, (char *)buf);
    } else {
        scoreColor.r = g;
        scoreColor.g = r;
        scoreColor.b = b;
        status = osal_snv_write(SNV_SCORE_COLOR_ID, SNV_BUF_LEN, (char *)buf);
    }

    if (status != SUCCESS) {
        Log_info1("snv write fail: %d", status);
    }

    uint16_t index = isTime ? 0 : NUM_TIME_LEDS;
    uint16_t end = isTime ? NUM_TIME_LEDS : NUM_SCORE_LEDS;

    for (uint16_t i = index; i < end; i++) {
        WS2812_setLEDcolor(index, g, r, b, WS2812_NOREFRESH);
    }

    WS2812_refreshLEDs();

}

void RGBLED_UpdateTimeDigits(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5) {
/*
    RGBLED_UpdateDigits(0, digit1, 0, true); // min 1
    RGBLED_UpdateDigits(7 * LED_IN_SEG, digit2, 0, true); // min 2
    RGBLED_UpdateDigits((14 * LED_IN_SEG), digit3, 2, true); // sec 1
    RGBLED_UpdateDigits((21 * LED_IN_SEG), digit4, 2, true); // sec 2
    RGBLED_UpdateDigits((28 * LED_IN_TIME_SEG), digit5, 3, true); // 1/10 sec
*/
    if (digit4 == 0) {
        for (uint8_t i = 0; i < Board_WS2812_NUM_LEDS; i++) {
            WS2812_setLEDcolor(i, 0, 0, 0, WS2812_NOREFRESH);
        }
    }

    // testing with launchpad
    if (digit4 > 0 && digit4 <= Board_WS2812_NUM_LEDS) {
        WS2812_setLEDcolor(digit4 - 1, timeColor.r, timeColor.g, timeColor.b, WS2812_NOREFRESH);
    }

    WS2812_refreshLEDs();
}

void RGBLED_UpdateScoreDigits(uint8_t digit1, uint8_t digit2) {
//    RGBLED_UpdateDigits((35 * LED_IN_SEG), digit1, 5, false);
//    RGBLED_UpdateDigits((35 * LED_IN_TIME_SEG) + (7 * LED_IN_SCORE_SEG), digit2, 5, false);

    if (digit2 == 0) {
        for (uint8_t i = 0; i < Board_WS2812_NUM_LEDS; i++) {
            WS2812_setLEDcolor(i, 0, 0, 0, WS2812_NOREFRESH);
        }
    }

    // testing with launchpad
    if (digit2 > 0 && digit2 <= Board_WS2812_NUM_LEDS) {
        WS2812_setLEDcolor(digit2 - 1, scoreColor.r, scoreColor.g, scoreColor.b, WS2812_NOREFRESH);
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

    uint16_t leds_in_seg = (!isTime) ? LED_IN_SCORE_SEG : LED_IN_TIME_SEG;
    // color for score and time, respectively
    color scoreTimeColor = (!isTime) ? scoreColor : timeColor;

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

    WS2812_setLEDcolor(14 * LED_IN_TIME_SEG, timeColor.r, timeColor.g, timeColor.b, WS2812_NOREFRESH);    // first dot after two digits
    WS2812_setLEDcolor((14 * LED_IN_TIME_SEG) + 1, timeColor.r, timeColor.g, timeColor.b, WS2812_NOREFRESH);  // second dot after two digits
    WS2812_setLEDcolor((14 * LED_IN_TIME_SEG) + 2, timeColor.r, timeColor.g, timeColor.b, WS2812_NOREFRESH);  // seconds dot after 4 digits/2 dots

    WS2812_refreshLEDs();
#endif

  RGBLED_SetParameter(RGB_LED_PARAM_COLOR, sizeof(LEDs), LEDs);
}

#endif // Board_RGB_NUM_LEDS > 0

/*********************************************************************
*********************************************************************/
