/**************************************************************************/
/*!
    @file     Adafruit_TCS34725.cpp
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "Adafruit_TCS34725.h"
#include <os.h>
extern tcs34725 htcs34725_eval;
extern I2C_HandleTypeDef hi2c1;

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float powf(const float x, const float y)
{
	return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/

void tcs34725_write8(uint8_t reg)
{
	HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS_WRITE, &reg, 1, 50);
}


void tcs34725_write16(uint8_t reg, uint8_t value)
{
	uint8_t command[2];
	command[0] = reg;
	command[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS_WRITE, command, 2, 50);
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t tcs34725_read8(uint8_t reg)
{
	uint8_t x;
	HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS_WRITE, &reg, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS_READ, &x, 1, 50);

	return x;
}

uint16_t tcs34725_read16(uint8_t reg)
{
	uint8_t data[2];
	uint16_t x;


	HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS_WRITE, &reg, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS_READ, data, 2, 50);
	x = data[0] | (data[1] << 8);
	return x;
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void tcs34725_enable(void)
{
        OS_ERR err = OS_ERR_NONE;
	tcs34725_write16(TCS34725_COMMAND_BIT | TCS34725_ENABLE, TCS34725_ENABLE_PON);
	OSTimeDly(3, OS_OPT_TIME_DLY, &err);
	tcs34725_write16(TCS34725_COMMAND_BIT | TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void tcs34725_disable(void)
{
	/* Turn the device off to save power */
	uint8_t reg = 0;
	reg = tcs34725_read8(TCS34725_COMMAND_BIT | TCS34725_ENABLE);
	tcs34725_write16(TCS34725_COMMAND_BIT | TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}


/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
bool tcs34725_begin(void)
{
	/* Make sure we're actually connected */
	uint8_t x = tcs34725_read8(TCS34725_COMMAND_BIT | TCS34725_ID);
	if ((x != 0x44) )
	{
		printf("Cannot find TCS3472\r\n");
		return false;
	}
	printf("Found TCS3472\r\n");
	htcs34725_eval._initialized = true;

	/* Set default integration time and gain */
	tcs34725_setIntegrationTime(htcs34725_eval._integration);
	tcs34725_setGain(htcs34725_eval._gain);

	/* Note: by default, the device is in power down mode on bootup */
	tcs34725_enable();

	return true;
}
  
/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void tcs34725_setIntegrationTime(tcs34725IntegrationTime_t it)
{
	if (!htcs34725_eval._initialized)
	{
		if (!tcs34725_begin())
		{
			return;
		}
	}
	/* Update value placeholders */
	htcs34725_eval._integration = it;

	/* Update the timing register */
	tcs34725_write16(TCS34725_COMMAND_BIT | TCS34725_ATIME, it);

}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void tcs34725_setGain(tcs34725Gain_t gain)
{

	if (!htcs34725_eval._initialized)
	{
		if (!tcs34725_begin())
		{
			return;
		}
	}

	/* Update value placeholders */
	htcs34725_eval._gain = gain;

	/* Update the timing register */
	tcs34725_write16(TCS34725_COMMAND_BIT | TCS34725_CONTROL, gain);

}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void tcs34725_getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
        OS_ERR err = OS_ERR_NONE;
        
	if (!htcs34725_eval._initialized)
	{
		if (!tcs34725_begin())
		{
			return;
		}
	}

	*r = 15;
	*g = 15;
	*b = 15;
	*c = tcs34725_read16(TCS34725_COMMAND_BIT | TCS34725_CDATAL);
	*r = tcs34725_read16(TCS34725_COMMAND_BIT | TCS34725_RDATAL);
	*g = tcs34725_read16(TCS34725_COMMAND_BIT | TCS34725_GDATAL);
	*b = tcs34725_read16(TCS34725_COMMAND_BIT | TCS34725_BDATAL);





  /* Set a delay for the integration time */
  switch (htcs34725_eval._integration)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
		
        OSTimeDly(3, OS_OPT_TIME_DLY, &err);
	break;
    case TCS34725_INTEGRATIONTIME_24MS:
    	OSTimeDly(24, OS_OPT_TIME_DLY, &err);
    	break;
    case TCS34725_INTEGRATIONTIME_50MS:
    	OSTimeDly(50, OS_OPT_TIME_DLY, &err);
    	break;
    case TCS34725_INTEGRATIONTIME_101MS:
    	OSTimeDly(101, OS_OPT_TIME_DLY, &err);
    	break;
    case TCS34725_INTEGRATIONTIME_154MS:
    	OSTimeDly(154, OS_OPT_TIME_DLY, &err);
    	break;
    case TCS34725_INTEGRATIONTIME_700MS:
    	OSTimeDly(700, OS_OPT_TIME_DLY, &err);
    	break;
  }
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t tcs34725_calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to lux
*/
/**************************************************************************/
uint16_t tcs34725_calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}


void tcs34725_setInterrupt(bool i)
{
	tcs34725_enable();
	uint8_t r = tcs34725_read8(TCS34725_COMMAND_BIT | TCS34725_ENABLE);

	if (i)
	{
		r |= TCS34725_ENABLE_AIEN;
	}
	else
	{
		r &= ~TCS34725_ENABLE_AIEN;
	}

	tcs34725_write16(TCS34725_COMMAND_BIT | TCS34725_ENABLE, r);
	tcs34725_disable();
}

void tcs34725_clearInterrupt(void)
{
	tcs34725_enable();
	tcs34725_write8(TCS34725_COMMAND_BIT | 0x66);
	tcs34725_disable();
}


void tcs34725_setIntLimits(uint16_t low, uint16_t high)
{
	tcs34725_enable();
	tcs34725_write16(TCS34725_COMMAND_BIT | 0x04, low & 0xFF);
	tcs34725_write16(TCS34725_COMMAND_BIT | 0x05, low >> 8);
	tcs34725_write16(TCS34725_COMMAND_BIT | 0x06, high & 0xFF);
	tcs34725_write16(TCS34725_COMMAND_BIT | 0x07, high >> 8);
	tcs34725_disable();
}

