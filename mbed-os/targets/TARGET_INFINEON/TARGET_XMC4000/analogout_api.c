/*
 * analogin_api.c
 *
 *  Created on: 06.04.2017
 *      Author: aamark
 */

#include "mbed_assert.h"
#include "analogout_api.h"

#if DEVICE_ANALOGOUT

#include "xmc_dac.h"
#include "pinmap.h"
#include "PeripheralPins.h"

void analogout_init(dac_t *obj, PinName pin)
{
	obj->dac = (DACName)pinmap_peripheral(pin, PinMap_DAC);
	MBED_ASSERT(obj->dac != (DACName)NC);

	//4bit group, 4bit channel
	obj->channel_no = pinmap_function(pin, PinMap_DAC);
	MBED_ASSERT(obj->channel_no != (uint32_t)NC);

	XMC_DAC_CH_CONFIG_t channel_config;
	memset(&channel_config, 0, sizeof(channel_config));
	XMC_DAC_CH_Init((XMC_DAC_t*)obj->dac, obj->channel_no, &channel_config);

	XMC_DAC_CH_StartSingleValueMode((XMC_DAC_t*)obj->dac, obj->channel_no);
	XMC_DAC_CH_Write((XMC_DAC_t*)obj->dac, obj->channel_no, 0);
}

void analogout_write(dac_t *obj, float value)
{
	if (value > 1.0)
		value = 1.0;
	else if(value < 0.0)
		value = 0.0;

	uint16_t dac_val = value * 0xFFFF;

	XMC_DAC_CH_Write((XMC_DAC_t*)obj->dac, obj->channel_no, dac_val >> 4);
}

void analogout_write_u16(dac_t *obj, uint16_t value)
{
	XMC_DAC_CH_Write((XMC_DAC_t*)obj->dac, obj->channel_no, value >> 4);
}

float analogout_read(dac_t *obj)
{
	return (XMC_DAC_CH_GetRampStart((XMC_DAC_t*)obj->dac, obj->channel_no) << 4)/(float)0xFFFF;
}

uint16_t analogout_read_u16(dac_t *obj)
{
	return XMC_DAC_CH_GetRampStart((XMC_DAC_t*)obj->dac, obj->channel_no) << 4;
}

#endif
