/*
 * analogin_api.c
 *
 *  Created on: 06.04.2017
 *      Author: aamark
 */

#include "mbed_assert.h"
#include "analogin_api.h"

#if DEVICE_ANALOGIN

#include "xmc_vadc.h"
#include "pinmap.h"
#include "PeripheralPins.h"

bool m_vadc_initialized = false;

void analogin_init(analogin_t *obj, PinName pin)
{
	obj->adc = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
	MBED_ASSERT(obj->adc != (ADCName)NC);

	//4bit group, 4bit channel
	uint8_t function = pinmap_function(pin, PinMap_ADC);
	MBED_ASSERT(function != (uint32_t)NC);

	obj->group_no = (function & 0xF0) >> 4;
	obj->channel_no = function & 0x0F;

	if (!m_vadc_initialized)
	{
		XMC_VADC_GLOBAL_CONFIG_t global_config;
		memset(&global_config, 0, sizeof(global_config));
		XMC_VADC_GLOBAL_Init(VADC, &global_config);

		XMC_VADC_GROUP_CONFIG_t group_config;
		memset(&group_config, 0, sizeof(group_config));

		/* Initialize all the Groups */
		XMC_VADC_GROUP_Init((XMC_VADC_GROUP_t*)VADC_0, &group_config);
		XMC_VADC_GROUP_Init((XMC_VADC_GROUP_t*)VADC_1, &group_config);
		XMC_VADC_GROUP_Init((XMC_VADC_GROUP_t*)VADC_2, &group_config);
		XMC_VADC_GROUP_Init((XMC_VADC_GROUP_t*)VADC_3, &group_config);

		XMC_VADC_GROUP_SetPowerMode((XMC_VADC_GROUP_t*)VADC_0, XMC_VADC_GROUP_POWERMODE_NORMAL);
		XMC_VADC_GROUP_SetPowerMode((XMC_VADC_GROUP_t*)VADC_1, XMC_VADC_GROUP_POWERMODE_NORMAL);
		XMC_VADC_GROUP_SetPowerMode((XMC_VADC_GROUP_t*)VADC_2, XMC_VADC_GROUP_POWERMODE_NORMAL);
		XMC_VADC_GROUP_SetPowerMode((XMC_VADC_GROUP_t*)VADC_3, XMC_VADC_GROUP_POWERMODE_NORMAL);

		XMC_VADC_GLOBAL_StartupCalibration(VADC);

		XMC_VADC_GLOBAL_CLASS_t iclass_config;
		memset(&iclass_config, 0, sizeof(iclass_config));
		XMC_VADC_GLOBAL_InputClassInit(VADC, iclass_config, XMC_VADC_GROUP_CONV_STD, 0);

		XMC_VADC_BACKGROUND_CONFIG_t backgnd_config;
		memset(&backgnd_config, 0, sizeof(backgnd_config));
		backgnd_config.enable_auto_scan = 1;
		XMC_VADC_GLOBAL_BackgroundInit(VADC, &backgnd_config);
	}

	XMC_VADC_CHANNEL_CONFIG_t channel_config;
	memset(&channel_config, 0, sizeof(channel_config));
	channel_config.input_class = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0;
	channel_config.result_reg_number = obj->channel_no;
	channel_config.result_alignment = XMC_VADC_RESULT_ALIGN_RIGHT;
	channel_config.alias_channel = -1;
	XMC_VADC_GROUP_ChannelInit((XMC_VADC_GROUP_t*)obj->adc, obj->channel_no, &channel_config);

	XMC_VADC_RESULT_CONFIG_t res_config;
	memset(&res_config, 0, sizeof(res_config));
	XMC_VADC_GROUP_ResultInit((XMC_VADC_GROUP_t*)obj->adc, obj->channel_no, &res_config);

	XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, obj->group_no, obj->channel_no);

	if (!m_vadc_initialized)
	{
		XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);
		m_vadc_initialized = true;
	}
}

float analogin_read(analogin_t *obj)
{
	return XMC_VADC_GROUP_GetResult((XMC_VADC_GROUP_t*)obj->adc, obj->channel_no)/4095.0;
}

uint16_t analogin_read_u16(analogin_t *obj)
{
	return XMC_VADC_GROUP_GetResult((XMC_VADC_GROUP_t*)obj->adc, obj->channel_no) << 4;
}



#endif
