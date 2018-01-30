/*
 * gpio_api.c
 *
 *  Created on: 03.04.2017
 *      Author: aamark
 */

#include "gpio_api.h"
#include "PeripheralPins.h"

void gpio_init_out(gpio_t* gpio, PinName pin)
{
	gpio_init_out_ex(gpio, pin, 0);
}

void gpio_init_out_ex(gpio_t* gpio, PinName pin, int value)
{
	gpio_init_port(gpio, pin);

	XMC_GPIO_CONFIG_t config =
	{
		.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
	};

	if (value)
		config.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
	else
		config.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW;

	XMC_GPIO_Init(gpio->port, gpio->pin, &config);

}

void gpio_init_in(gpio_t* gpio, PinName pin)
{
	gpio_init_in_ex(gpio, pin, PullDefault);
}

void gpio_init_in_ex(gpio_t* gpio, PinName pin, PinMode mode)
{
	gpio_init_port(gpio, pin);

	XMC_GPIO_CONFIG_t config;

	switch (mode)
	{
		case PullNone:
			config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
			break;
		case PullDown:
			config.mode = XMC_GPIO_MODE_INPUT_PULL_DOWN;
			break;
		case PullUp:
			config.mode = XMC_GPIO_MODE_INPUT_PULL_UP;
			break;
	}

	XMC_GPIO_Init(gpio->port, gpio->pin, &config);
}

void gpio_mode(gpio_t *obj, PinMode mode)
{
	XMC_GPIO_MODE_t xmc_mode;

	switch (mode)
	{
		case PullNone:
			xmc_mode = XMC_GPIO_MODE_INPUT_TRISTATE;
			break;
		case PullDown:
			xmc_mode = XMC_GPIO_MODE_INPUT_PULL_DOWN;
			break;
		case PullUp:
			xmc_mode = XMC_GPIO_MODE_INPUT_PULL_UP;
			break;
	}

	XMC_GPIO_SetMode(obj->port, obj->pin, xmc_mode);
}

void gpio_init_inout(gpio_t* gpio, PinName pin, PinDirection direction, PinMode mode, int value)
{
	XMC_GPIO_CONFIG_t config;

	gpio_init_port(gpio, pin);

	if (direction == PIN_INPUT)
	{
		switch (mode)
		{
			case PullNone:
				config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
				break;
			case PullDown:
				config.mode = XMC_GPIO_MODE_INPUT_PULL_DOWN;
				break;
			case PullUp:
				config.mode = XMC_GPIO_MODE_INPUT_PULL_UP;
				break;
		}
	}
	else
	{
		config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
		if (value)
			config.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
		else
			config.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW;
	}

	XMC_GPIO_Init(gpio->port, gpio->pin, &config);
}

void gpio_dir(gpio_t *obj, PinDirection direction)
{
	XMC_GPIO_MODE_t mode;

	if (direction == PIN_INPUT)
		mode = XMC_GPIO_MODE_INPUT_PULL_UP;
	else
		mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;

	XMC_GPIO_SetMode(obj->port, obj->pin, mode);
}
