/* Includes ------------------------------------------------------------------*/


#include <stdint.h>
#include "Adafruit_TCS34725.h"
#include "stm32469i_discovery.h"
#include "stm32469i_discovery_ts.h"
#include <os.h>

#ifndef TS_MULTI_TOUCH_SUPPORTED
#define TS_MULTI_TOUCH_SUPPORTED
#endif

/* Global variables ---------------------------------------------------------*/
TS_StateTypeDef  TS_State = {0};

extern tcs34725 htcs34725_eval;

void Touchscreen_demo1(void)
{
	uint16_t x1, y1;
	//uint8_t exitTsUseCase = 0;
	uint32_t ts_status = TS_OK;
        uint8_t lcd_string[40] = "";
        /* Reset touch data information */
        TS_State.touchDetected = 0;
        TS_State.touchX[0] = 0;
        TS_State.touchY[0] = 0;
        
        OS_ERR err = OS_ERR_NONE;
        
        /* If calibration is not yet done, proceed with calibration */
	if (TouchScreen_IsCalibrationDone() == 0)
	{
	  ts_status = Touchscreen_Calibration();
		if(ts_status == TS_OK)
		{
			BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"Touchscreen calibration success.", CENTER_MODE);
		}
	} /* of if (TouchScreen_IsCalibrationDone() == 0) */


				/* Check in polling mode in touch screen the touch status and coordinates */
				/* of touches if touch occurred                                           */
                                ts_status = BSP_TS_GetState(&TS_State);
				if(TS_State.touchDetected)
				{
					/* One or dual touch have been detected          */
					/* Only take into account the first touch so far */

				  /* Get X and Y position of the first touch post calibrated */
					x1 = TouchScreen_Get_Calibrated_X(TS_State.touchX[0]);
					y1 = TouchScreen_Get_Calibrated_Y(TS_State.touchY[0]);
                                 
                                        if ((y1 > 360u) && (y1 < 408u))
					{
						if ((x1 > 0u) && (x1 < 272u))
						{
                                                  switch (htcs34725_eval._integration)
                                                  {

                                                    case TCS34725_INTEGRATIONTIME_50MS:
                                                        tcs34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_101MS);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                    case TCS34725_INTEGRATIONTIME_101MS:
                                                        tcs34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                    case TCS34725_INTEGRATIONTIME_154MS:
                                                        tcs34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_700MS);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                    case TCS34725_INTEGRATIONTIME_700MS:
                                                        tcs34725_setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                  }
                                                    
                                                }
                                        }
                                        if ((y1 > 432u) && (y1 < 480u))
					{
						if ((x1 > 0u) && (x1 < 68u))
						{
                                                  switch (htcs34725_eval._gain)
                                                  {

                                                    case TCS34725_GAIN_1X:
                                                        tcs34725_setGain(TCS34725_GAIN_4X);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                    case TCS34725_GAIN_4X:
                                                        tcs34725_setGain(TCS34725_GAIN_16X);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                    case TCS34725_GAIN_16X:
                                                        tcs34725_setGain(TCS34725_GAIN_60X);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                    case TCS34725_GAIN_60X:
                                                        tcs34725_setGain(TCS34725_GAIN_1X);
                                                        OSTimeDly(300, OS_OPT_TIME_DLY, &err);
                                                        break;
                                                  }
                                                }
                                        }
                                        /*
                                        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
                                        BSP_LCD_SetFont(&Font12);
                                        sprintf((char*)lcd_string, "x1 = %6d, y1 = %6d",
                                                x1,
                                                y1
                                                );
                                        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 45, lcd_string, CENTER_MODE);
                                        */
                                        
                                        

				} /* of if(TS_State.TouchDetected) */



			/* Wait for a key button press to switch to next test case of BSP validation application */
			/* Otherwise stay in the test */

			OSTimeDly(20, OS_OPT_TIME_DLY, &err);


}