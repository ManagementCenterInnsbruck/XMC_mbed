/*
 * analogin_api.c
 *
 *  Created on: 06.04.2017
 *      Author: aamark
 */

#include "mbed_assert.h"
#include "pwmout_api.h"

#if DEVICE_PWMOUT

#include <math.h>
#include "xmc_ccu4.h"
#include "pinmap.h"
#include "PeripheralPins.h"

void pwmout_init(pwmout_t *obj, PinName pin)
{
	uint8_t alt_no;

	obj->pwm = (PWMName)pinmap_peripheral(pin, PinMap_PWM);
	MBED_ASSERT(obj->pwm != (PWMName)NC);

	uint8_t function = pinmap_function(pin, PinMap_PWM);
	MBED_ASSERT(function != (uint32_t)NC);

	alt_no = function & 0x0F;
	obj->slice_no = (function & 0xF0) >> 4;

	if (obj->pwm >= (int)CCU42_CC40)
		obj->global = (int)CCU42;
	else if (obj->pwm >= (int)CCU41_CC40)
		obj->global = (int)CCU41;
	else
		obj->global = (int)CCU40;


	XMC_CCU4_Init((XMC_CCU4_MODULE_t*)obj->global, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);

	XMC_CCU4_SLICE_COMPARE_CONFIG_t slice_config;
	memset(&slice_config, 0, sizeof(slice_config));

	obj->prescaler = 0x00;
	obj->period = 0x7FFF;
	obj->compare = obj->period+1;

	XMC_CCU4_SLICE_CompareInit((XMC_CCU4_SLICE_t*)obj->pwm, &slice_config);
	XMC_CCU4_SLICE_SetTimerPeriodMatch((XMC_CCU4_SLICE_t*)obj->pwm, obj->period);
	XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)obj->pwm, obj->compare);

	XMC_CCU4_EnableShadowTransfer((XMC_CCU4_MODULE_t*)obj->global, (1 << 4*obj->slice_no));

	while (((XMC_CCU4_MODULE_t*)obj->global)->GCST & (1 << 4*obj->slice_no))
		__NOP();

	gpio_t gpio;
	gpio_init_port(&gpio, pin);

	XMC_GPIO_CONFIG_t gpio_conf =
	{
		.mode			= XMC_GPIO_MODE_OUTPUT_PUSH_PULL | (alt_no << PORT0_IOCR0_PC0_Pos),
		.output_level	= XMC_GPIO_OUTPUT_LEVEL_HIGH,
	};

	XMC_GPIO_Init(gpio.port, gpio.pin, &gpio_conf);

	XMC_CCU4_EnableClock((XMC_CCU4_MODULE_t*)obj->global, obj->slice_no);
	XMC_CCU4_SLICE_StartTimer((XMC_CCU4_SLICE_t*)obj->pwm);
}

void pwmout_write(pwmout_t *obj, float percent)
{
	if (percent > 1.0)
		percent = 1.0;
	else if (percent < 0.0)
		percent = 0.0;

	obj->compare = (obj->period+1) * (1-percent);

	XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)obj->pwm, obj->compare);
	XMC_CCU4_EnableShadowTransfer((XMC_CCU4_MODULE_t*)obj->global, (1 << 4*obj->slice_no));

	while (((XMC_CCU4_MODULE_t*)obj->global)->GCST & (1 << 4*obj->slice_no))
		__NOP();
}

float pwmout_read(pwmout_t *obj)
{
	return 1.0 - (float)obj->compare/(obj->period+1);
}

void pwmout_period_us(pwmout_t *obj, int us)
{
	uint8_t prescaler = 0;

	float duty = 1.0 - ((float)obj->compare/(obj->period+1));

	while (us > (float)0xFFFF * (pow(2, prescaler) / (float)CLOCK_FREQ))
		prescaler++;

	obj->prescaler = prescaler;
	obj->period = ((float)CLOCK_FREQ  * us) / (pow(2,prescaler));
	obj->compare = (obj->period + 1) * (1-duty);

	XMC_CCU4_SLICE_SetPrescaler((XMC_CCU4_SLICE_t*)obj->pwm, obj->prescaler);
	XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)obj->pwm, obj->compare);
	XMC_CCU4_SLICE_SetTimerPeriodMatch((XMC_CCU4_SLICE_t*)obj->pwm, obj->period);

	XMC_CCU4_EnableShadowTransfer((XMC_CCU4_MODULE_t*)obj->global, (1 << 4*obj->slice_no));

	while (((XMC_CCU4_MODULE_t*)obj->global)->GCST & (1 << 4*obj->slice_no))
		__NOP();
}

void pwmout_period_ms(pwmout_t *obj, int ms)
{
	pwmout_period_us(obj, ms * 1000);
}

void pwmout_period(pwmout_t *obj, float seconds)
{
	pwmout_period_us(obj, seconds * 1000000);
}

void pwmout_pulsewidth_us(pwmout_t *obj, int us)
{
	float period_us = pow(2, obj->prescaler) / (float)CLOCK_FREQ * obj->period;

	if (us > period_us)
		obj->compare = obj->period + 1;
	else
		obj->compare = (obj->period + 1) * (1 - us / period_us);

	XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)obj->pwm, obj->compare);
	XMC_CCU4_EnableShadowTransfer((XMC_CCU4_MODULE_t*)obj->global, (1 << 4*obj->slice_no));

	while (((XMC_CCU4_MODULE_t*)obj->global)->GCST & (1 << 4*obj->slice_no))
		__NOP();
}

void pwmout_pulsewidth_ms(pwmout_t *obj, int ms)
{
	pwmout_pulsewidth_us(obj, 1000 * ms);
}

void pwmout_pulsewidth(pwmout_t *obj, float seconds)
{
	pwmout_pulsewidth_us(obj, seconds * 1000000);
}

#endif
