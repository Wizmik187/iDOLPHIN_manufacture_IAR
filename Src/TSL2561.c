/*
 * TSL2561.C

 *
 *  Created on: 2016. 8. 10.
 *      Author: user
 */

// lux equation approximation without floating point calculations
//////////////////////////////////////////////////////////////////////////////
// Routine: unsigned int CalculateLux(unsigned int ch0, unsigned int ch0, int iType)
//
// Description: Calculate the approximate illuminance (lux) given the raw
// channel values of the TSL2560. The equation if implemented
// as a piece-wise linear approximation.
//
// Arguments: unsigned int iGain - gain, where 0:1X, 1:16X
// unsigned int tInt - integration time, where 0:13.7mS, 1:100mS, 2:402mS,
// 3:Manual
// unsigned int ch0 - raw channel value from channel 0 of TSL2560
// unsigned int ch1 - raw channel value from channel 1 of TSL2560
// unsigned int iType - package type (T or CS)
//
// Return: unsigned int - the approximate illuminance (lux)
//
//////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include "TSL2561.h"


uint32_t CalculateLux(uint8_t iGain, uint8_t tInt, uint16_t ch0, uint16_t ch1, uint8_t iType)
{
	//------------------------------------------------------------------------
	// first, scale the channel values depending on the gain and integration time
	// 16X, 402mS is nominal.
	// scale if integration time is NOT 402 msec
	uint16_t chScale;
	uint32_t channel1;
	uint32_t channel0;

	switch (tInt)
	{
		case 0: // 13.7 msec
			chScale = CHSCALE_TINT0;
			break;
		case 1: // 101 msec
			chScale = CHSCALE_TINT1;
			break;
		default: // assume no scaling
			chScale = (1 << CH_SCALE);
		break;
	}
	// scale if gain is NOT 16X
	if (!iGain) chScale = chScale << 4; // scale 1X to 16X
	// scale the channel values
	channel0 = (ch0 * chScale) >> CH_SCALE;
	channel1 = (ch1 * chScale) >> CH_SCALE;
	//------------------------------------------------------------------------
	// find the ratio of the channel values (Channel1/Channel0)
	// protect against divide by zero
	uint32_t ratio1 = 0;

	if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;
	// round the ratio value
	uint32_t ratio = (ratio1 + 1) >> 1;
	// is ratio <= eachBreak ?
	uint16_t b, m;
	switch (iType)
	{
		case 0: // T, FN and CL package
			if ((ratio >= 0) && (ratio <= K1T))
			{b=B1T; m=M1T;}
			else if (ratio <= K2T)
			{b=B2T; m=M2T;}
			else if (ratio <= K3T)
			{b=B3T; m=M3T;}
			else if (ratio <= K4T)
			{b=B4T; m=M4T;}
			else if (ratio <= K5T)
			{b=B5T; m=M5T;}
			else if (ratio <= K6T)
			{b=B6T; m=M6T;}
			else if (ratio <= K7T)
			{b=B7T; m=M7T;}
			else if (ratio > K8T)
			{b=B8T; m=M8T;}
			break;
		case 1:// CS package
			if ((ratio >= 0) && (ratio <= K1C))
			{b=B1C; m=M1C;}
			else if (ratio <= K2C)
			{b=B2C; m=M2C;}
			else if (ratio <= K3C)
			{b=B3C; m=M3C;}
			else if (ratio <= K4C)
			{b=B4C; m=M4C;}
			else if (ratio <= K5C)
			{b=B5C; m=M5C;}
			else if (ratio <= K6C)
			{b=B6C; m=M6C;}
			else if (ratio <= K7C)
			{b=B7C; m=M7C;}
			else if (ratio > K8C)
			{b=B8C; m=M8C;}
			break;
	}

	uint32_t temp;
	temp = ((channel0 * b) - (channel1 * m));
	// do not allow negative lux value
	if (temp < 0) temp = 0;
	// round lsb (2^(LUX_SCALE-1))
	temp += (1 << (LUX_SCALE-1));
	// strip off fractional portion
	uint32_t lux = temp >> LUX_SCALE;

	return(lux);
}
