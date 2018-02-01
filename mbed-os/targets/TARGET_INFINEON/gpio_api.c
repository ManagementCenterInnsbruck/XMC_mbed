/*
 * gpio_api.c
 *
 *  Created on: 03.04.2017
 *      Author: aamark
 */

#include "gpio_api.h"
#include "PeripheralPins.h"

void gpio_init(gpio_t *obj, PinName pin)
{
	obj->name = pin;
	obj->port = (XMC_GPIO_PORT_t*)(PORT0_BASE + ((pin & 0xF0) << 4));
	obj->pin = pin & 0x0F;
	obj->dir = PIN_INPUT;
}

void gpio_mode(gpio_t *obj, PinMode mode)
{
	if (obj->dir == PIN_INPUT)
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
}

void gpio_dir(gpio_t *obj, PinDirection direction)
{
	XMC_GPIO_MODE_t mode;

	if (direction == PIN_INPUT)
		mode = XMC_GPIO_MODE_INPUT_PULL_UP;
	else
		mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;

	obj->dir = direction;

	XMC_GPIO_SetMode(obj->port, obj->pin, mode);
}
