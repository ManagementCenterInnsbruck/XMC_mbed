/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#include <stddef.h>
#include "cmsis.h"
#include "gpio_irq_api.h"
#include "pinmap.h"
#include "mbed_error.h"
#include "PeripheralPins.h"
#include "gpio_irq_device.h"

static gpio_irq_t* gpio_irqs[ERU_CHANNEL_NUM] = {0};
static uint32_t gpio_irq_ids[ERU_CHANNEL_NUM] = {0};

static gpio_irq_handler irq_handler;

static void handle_interrupt_in(uint8_t index)
{
	gpio_t gpio;

	if (gpio_irq_ids[index])
	{
		gpio_init_port(&gpio, gpio_irqs[index]->pin);

		if (gpio_read(&gpio))
			irq_handler(gpio_irq_ids[index], IRQ_RISE);
		else
			irq_handler(gpio_irq_ids[index], IRQ_FALL);
	}
}

void ERU0_0_IRQHandler()
{
	handle_interrupt_in(0);
}

void ERU0_1_IRQHandler()
{
	handle_interrupt_in(1);
}

void ERU0_2_IRQHandler()
{
	handle_interrupt_in(2);
}

void ERU0_3_IRQHandler()
{
	handle_interrupt_in(3);
}

void ERU1_0_IRQHandler()
{
	handle_interrupt_in(4);
}

void ERU1_1_IRQHandler()
{
	handle_interrupt_in(5);
}

void ERU1_2_IRQHandler()
{
	handle_interrupt_in(6);
}

void ERU1_3_IRQHandler()
{
	handle_interrupt_in(7);
}

/** Initialize the GPIO IRQ pin
 *
 * @param obj     The GPIO object to initialize
 * @param pin     The GPIO pin name
 * @param handler The handler to be attached to GPIO IRQ
 * @param id      The object ID (id != 0, 0 is reserved)
 * @return -1 if pin is NC, 0 otherwise
 */
int gpio_irq_init(gpio_irq_t *obj, PinName pin, gpio_irq_handler handler, uint32_t id)
{
	if (pin == NC) return -1;

	uint8_t index = ((pin & 0xF0) >> 4) * PORT_PIN_NUM + (pin & 0x0f);

	obj->pin = pin;
	obj->eru = (uint32_t*)pin_lines_desc[index].eru;
	obj->etl = pin_lines_desc[index].channel;

	if (obj->eru == (uint32_t*)ERU0)
	{
		gpio_irqs[obj->etl] = obj;
		gpio_irq_ids[obj->etl] = id;
		obj->irq_n = obj->etl + 1;
	}
	else
	{
		gpio_irqs[obj->etl+4] = obj;
		gpio_irq_ids[obj->etl+4] = id;
		obj->irq_n = obj->etl + 5;
	}

	XMC_ERU_ETL_CONFIG_t etl_config =
	{
		.input_a 				= pin_lines_desc[index].input,
		.input_b 				= pin_lines_desc[index].input,
		.enable_output_trigger 	= 1,
		.edge_detection 		= XMC_ERU_ETL_EDGE_DETECTION_DISABLED,
		.output_trigger_channel = pin_lines_desc[index].channel,
		.source					= pin_lines_desc[index].source,
	};

	XMC_ERU_ETL_Init((XMC_ERU_t*)obj->eru, obj->etl, &etl_config);
	XMC_ERU_OGU_SetServiceRequestMode((XMC_ERU_t*)obj->eru, obj->etl, XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER);

	irq_handler = handler;

	gpio_irq_enable(obj);

	return 0;
}

/** Release the GPIO IRQ PIN
 *
 * @param obj The gpio object
 */
void gpio_irq_free(gpio_irq_t *obj)
{
	for (uint8_t i=0; i<ERU_CHANNEL_NUM; i++)
	{
		if (obj == gpio_irqs[i])
		{
			gpio_irqs[i] = 0;
			gpio_irq_ids[i] = 0;
		}
	}
}

/** Enable/disable pin IRQ event
 *
 * @param obj    The GPIO object
 * @param event  The GPIO IRQ event
 * @param enable The enable flag
 */
void gpio_irq_set(gpio_irq_t *obj, gpio_irq_event event, uint32_t enable)
{
	XMC_ERU_ETL_EDGE_DETECTION_t edge = XMC_ERU_ETL_GetEdgeDetection((XMC_ERU_t*)obj->eru, obj->etl);

	if (event == IRQ_RISE)
	{
		if (enable)
			edge |= XMC_ERU_ETL_EDGE_DETECTION_RISING;
		else
			edge &= ~XMC_ERU_ETL_EDGE_DETECTION_RISING;
	}

	if (event == IRQ_FALL)
	{
		if (enable)
			edge |= XMC_ERU_ETL_EDGE_DETECTION_FALLING;
		else
			edge &= ~XMC_ERU_ETL_EDGE_DETECTION_FALLING;
	}

	XMC_ERU_ETL_SetEdgeDetection((XMC_ERU_t*)obj->eru, obj->etl, edge);
}

/** Enable GPIO IRQ
 *
 * This is target dependent, as it might enable the entire port or just a pin
 * @param obj The GPIO object
 */
void gpio_irq_enable(gpio_irq_t *obj)
{
	NVIC_ClearPendingIRQ(obj->irq_n);
	NVIC_EnableIRQ(obj->irq_n);
}

/** Disable GPIO IRQ
 *
 * This is target dependent, as it might disable the entire port or just a pin
 * @param obj The GPIO object
 */
void gpio_irq_disable(gpio_irq_t *obj)
{
	NVIC_DisableIRQ(obj->irq_n);
}
