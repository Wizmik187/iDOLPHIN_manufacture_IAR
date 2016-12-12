/**************************************************************************/
/*!
    @file     Adafruit_TSL2591.cpp
    @author   KT0WN (adafruit.com)

    This is a library for the Adafruit TSL2591 breakout board
    This library works with the Adafruit TSL2591 breakout
    ----> https://www.adafruit.com/products/1980

    Check out the links above for our tutorials and wiring diagrams
    These chips use I2C to communicate

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014 Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/



#include "stm32f4xx_hal.h"
#include "Adafruit_TSL2591.h"

extern tsl2591 htsl2591_eval;
extern I2C_HandleTypeDef hi2c1;

bool tsl2591_begin(void)
{
	
	uint8_t id = tsl2591_read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_ID);


	if (id == 0x50 )
	{
		printf("Found TSL2591\r\n");
	}
	else
	{
		printf("Cannot find TSL2591\r\n");
		return false;
	}
	
	htsl2591_eval._initialized = true;
	
	// Set default integration time and gain
	tsl2591_setTiming(htsl2591_eval._integration);
	tsl2591_setGain(htsl2591_eval._gain);
	
	// Note: by default, the device is in power down mode on bootup
	tsl2591_disable();

	return true;
}

void tsl2591_enable(void)
{
	if (!htsl2591_eval._initialized)
	{
		if (!tsl2591_begin())
		{
			return;
		}
	}
	
	// Enable the device by setting the control bit to 0x01
	tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
			TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN | TSL2591_ENABLE_NPIEN);

}

void tsl2591_disable(void)
{
	if (!htsl2591_eval._initialized)
	{
		if (!tsl2591_begin())
		{
			return;
		}
	}

	// Disable the device by setting the control bit to 0x00
	tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE, TSL2591_ENABLE_POWEROFF);

}


void tsl2591_setGain(tsl2591Gain_t gain)
{
	if (!htsl2591_eval._initialized)
	{
		if (!tsl2591_begin())
		{
			return;
		}
	}

	tsl2591_enable();
	htsl2591_eval._gain = gain;
	tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, htsl2591_eval._integration | htsl2591_eval._gain);
	tsl2591_disable();
}


void tsl2591_setTiming(tsl2591IntegrationTime_t integration)
{
	if (!htsl2591_eval._initialized)
	{
		if (!tsl2591_begin())
		{
			return;
		}
	}

	tsl2591_enable();
	htsl2591_eval._integration = integration;
	tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, htsl2591_eval._integration | htsl2591_eval._gain);
	tsl2591_disable();
}


uint32_t tsl2591_calculateLux(uint16_t ch0, uint16_t ch1)
{
  float    atime, again;
  float    cpl, lux1, lux2, lux;

  // Check for overflow conditions first
  if (htsl2591_eval._integration == TSL2591_INTEGRATIONTIME_100MS)
  {
	  if ((ch0 == 0x9401) || (ch1 == 0x9401))
	  {
		  return 0;
	  }
  }
  else
  {
	  if ((ch0 == 0xFFFF) || (ch1 == 0xFFFF))
	  {
	    // Signal an overflow
		  return 0;
	  }
  }


  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future

  switch (htsl2591_eval._integration)
  {
    case TSL2591_INTEGRATIONTIME_100MS :
      atime = 100.0F;
      break;
    case TSL2591_INTEGRATIONTIME_200MS :
      atime = 200.0F;
      break;
    case TSL2591_INTEGRATIONTIME_300MS :
      atime = 300.0F;
      break;
    case TSL2591_INTEGRATIONTIME_400MS :
      atime = 400.0F;
      break;
    case TSL2591_INTEGRATIONTIME_500MS :
      atime = 500.0F;
      break;
    case TSL2591_INTEGRATIONTIME_600MS :
      atime = 600.0F;
      break;
    default: // 100ms
      atime = 100.0F;
      break;
  }

  switch (htsl2591_eval._gain)
  {
    case TSL2591_GAIN_LOW :
      again = 1.0F;
      break;
    case TSL2591_GAIN_MED :
      again = 25.0F;
      break;
    case TSL2591_GAIN_HIGH :
      again = 428.0F;
      break;
    case TSL2591_GAIN_MAX :
      again = 9876.0F;
      break;
    default:
      again = 1.0F;
      break;
  }

  // cpl = (ATIME * AGAIN) / DF
  cpl = (atime * again) / TSL2591_LUX_DF;

  lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
  lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD * (float)ch1 ) ) / cpl;
  lux = lux1 > lux2 ? lux1 : lux2;

  // Alternate lux calculation
  //lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;

  // Signal I2C had no errors
  return (uint32_t)lux;
}

uint32_t tsl2591_getFullLuminosity (void)
{
	if (!htsl2591_eval._initialized)
	{
		if (!tsl2591_begin())
		{
			return 0;
		}
	}

	// Enable the device
	tsl2591_enable();

	// Wait x ms for ADC to complete
	for (uint8_t d=0; d<=htsl2591_eval._integration; d++)
	{
		HAL_Delay(120);
	}


	uint32_t x;

	x = tsl2591_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
	x <<= 16;
	x |= tsl2591_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);

	tsl2591_disable();

	return x;
}

uint16_t tsl2591_getLuminosity (uint8_t channel)
{
	uint32_t x = tsl2591_getFullLuminosity();

	if (channel == TSL2591_FULLSPECTRUM)
	{
		// Reads two byte value from channel 0 (visible + infrared)
		return (x & 0xFFFF);
	}
	else if (channel == TSL2591_INFRARED)
	{
		// Reads two byte value from channel 1 (infrared)
		return (x >> 16);
	}
	else if (channel == TSL2591_VISIBLE)
	{
		// Reads all and subtracts out just the visible!
		return ( (x & 0xFFFF) - (x >> 16));
	}

	// unknown channel!
	return 0;
}

void tsl2591_registerInterrupt(uint16_t lowerThreshold, uint16_t upperThreshold)
{
  if (!htsl2591_eval._initialized)
  {
    if (!tsl2591_begin())
    {
      return;
    }
  }

  tsl2591_enable();
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAILTL, lowerThreshold);
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAILTH, lowerThreshold >> 8);
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAIHTL, upperThreshold);
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_NPAIHTH, upperThreshold >> 8);
  tsl2591_disable();
}

void tsl2591_registerInterruptPersist(uint16_t lowerThreshold, uint16_t upperThreshold, tsl2591Persist_t persist)
{
  if (!htsl2591_eval._initialized)
  {
    if (!tsl2591_begin())
    {
      return;
    }
  }

  tsl2591_enable();
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_PERSIST_FILTER,  persist);
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTL, lowerThreshold);
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTH, lowerThreshold >> 8);
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTL, upperThreshold);
  tsl2591_write16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTH, upperThreshold >> 8);
  tsl2591_disable();
}

void tsl2591_clearInterrupt()
{
  if (!htsl2591_eval._initialized)
  {
    if (!tsl2591_begin())
    {
      return;
    }
  }

  tsl2591_enable();
  tsl2591_write8(TSL2591_CLEAR_INT);
  tsl2591_disable();
}


uint8_t tsl2591_getStatus()
{
  if (!htsl2591_eval._initialized)
  {
    if (!tsl2591_begin())
    {
      return 0;
    }
  }

  // Enable the device
  tsl2591_enable();
  uint8_t x;
  x = read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_STATUS);
  tsl2591_disable();
  return x;
}


uint8_t tsl2591_read8(uint8_t reg)
{

	uint8_t x;
	HAL_I2C_Master_Transmit(&hi2c1, TSL2591_ADDR_WRITE, &reg, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, TSL2591_ADDR_READ, &x, 1, 50);

	return x;

}

uint16_t tsl2591_read16(uint8_t reg)
{
  uint8_t data[2];
  uint16_t x;


  HAL_I2C_Master_Transmit(&hi2c1, TSL2591_ADDR_WRITE, &reg, 1, 50);
  HAL_I2C_Master_Receive(&hi2c1, TSL2591_ADDR_READ, data, 2, 50);

  x = data[0] | (data[1] << 8);


  return x;

}


void tsl2591_write8 (uint8_t reg)
{
	HAL_I2C_Master_Transmit(&hi2c1, TSL2591_ADDR_WRITE, &reg, 1, 50);
}


void tsl2591_write16 (uint8_t reg, uint8_t value)
{
	uint8_t command[2];
	command[0] = reg;
	command[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, TSL2591_ADDR_WRITE, command, 2, 50);
}


