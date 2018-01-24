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

#include "PeripheralPins.h"

// =====
// Note: Commented lines are alternative possibilities which are not used per default.
//       If you change them, you will have also to modify the corresponding xxx_api.c file
//       for pwmout, analogin, analogout, ...
// =====

void gpio_init_port(gpio_t* gpio, PinName pin)
{
	switch (pin & 0xF0)
	{
		case 0x00:
			gpio->port = (XMC_GPIO_PORT_t*) PORT0;
			break;
		case 0x10:
			gpio->port = (XMC_GPIO_PORT_t*) PORT1;
			break;
		case 0x20:
			gpio->port = (XMC_GPIO_PORT_t*) PORT2;
			break;
		case 0x30:
			gpio->port = (XMC_GPIO_PORT_t*) PORT3;
			break;
		case 0x40:
			gpio->port = (XMC_GPIO_PORT_t*) PORT4;
			break;
		case 0x50:
			gpio->port = (XMC_GPIO_PORT_t*) PORT5;
			break;
		case 0x60:
			gpio->port = (XMC_GPIO_PORT_t*) PORT6;
			break;
		case 0xe0:
			gpio->port = (XMC_GPIO_PORT_t*) PORT14;
			break;
		case 0xf0:
			gpio->port = (XMC_GPIO_PORT_t*) PORT15;
			break;
	}

	gpio->pin = pin & 0x0F;
	gpio->name = pin;
}

//*** DAC ***

const PinMap PinMap_DAC[] = {
		{p14_8, (int)DAC_0, 0x00},
		{p14_9, (int)DAC_0, 0x01},
};

//*** ADC ***

const PinMap PinMap_ADC[] = {
		{p14_0,  (int)VADC_0, 0x00},
		{p14_1,  (int)VADC_0, 0x01},
		{p14_2,  (int)VADC_0, 0x02},
		{p14_3,  (int)VADC_0, 0x03},
		{p14_4,  (int)VADC_0, 0x04},
		{p14_5,  (int)VADC_0, 0x05},
		{p14_6,  (int)VADC_0, 0x06},
		{p14_7,  (int)VADC_0, 0x07},
		{p14_8,  (int)VADC_1, 0x10},
		{p14_9,  (int)VADC_1, 0x11},
		{p14_12, (int)VADC_1, 0x14},
		{p14_13, (int)VADC_1, 0x15},
		{p14_14, (int)VADC_1, 0x16},
		{p14_15, (int)VADC_1, 0x17},
		{p15_2,  (int)VADC_2, 0x22},
		{p15_3,  (int)VADC_2, 0x23},
		{p15_4,  (int)VADC_2, 0x24},
		{p15_5,  (int)VADC_2, 0x25},
		{p15_6,  (int)VADC_2, 0x26},
		{p15_7,  (int)VADC_2, 0x27},
		{p15_8,  (int)VADC_3, 0x30},
		{p15_9,  (int)VADC_3, 0x31},
		{p15_12, (int)VADC_3, 0x34},
		{p15_13, (int)VADC_3, 0x35},
		{p15_14, (int)VADC_3, 0x36},
		{p15_15, (int)VADC_3, 0x37},
};

//*** PWM ***

const PinMap PinMap_PWM[] = {
		{p0_12, (int)PWM_4,  0x33},
		{p0_13, (int)PWM_3,  0x23},
		{p0_14, (int)PWM_2,  0x13},
		{p0_15, (int)PWM_1,  0x03},
		{p1_0,  (int)PWM_4,  0x33},
		{p1_1,  (int)PWM_3,  0x23},
		{p1_2,  (int)PWM_2,  0x13},
		{p1_3,  (int)PWM_1,  0x03},
		{p2_2,  (int)PWM_8,  0x33},
		{p2_3,  (int)PWM_7,  0x23},
		{p2_4,  (int)PWM_6,  0x13},
		{p2_5,  (int)PWM_5,  0x03},
		{p3_0,  (int)PWM_9,  0x03},
		{p3_3,  (int)PWM_12, 0x33},
		{p3_4,  (int)PWM_11, 0x23},
		{p3_5,  (int)PWM_10, 0x13},
		{p3_6,  (int)PWM_9,  0x03},
		{p3_7,  (int)PWM_8,  0x33},
		{p3_8,  (int)PWM_7,  0x23},
		{p3_9,  (int)PWM_6,  0x13},
		{p3_10, (int)PWM_5,  0x03},
		{p3_11, (int)PWM_12, 0x33},
		{p3_12, (int)PWM_11, 0x23},
		{p3_13, (int)PWM_10, 0x13},
};

//*** Serial ***

const PinMap PinMap_UART_RX[] = {
		{p0_0,  (int)UART_4, 0x13},
		{p0_4,  (int)UART_3, 0x00},
		{p0_5,  (int)UART_3, 0x01},
		{p1_4,  (int)UART_1, 0x01},
		{p1_5,  (int)UART_1, 0x00},
		{p2_2,  (int)UART_2, 0x10},
		{p2_5,  (int)UART_2, 0x11},
		{p2_14, (int)UART_3, 0x03},
		{p2_15, (int)UART_3, 0x02},
		{p3_4,  (int)UART_6, 0x11},
		{p4_0,  (int)UART_6, 0x12},
		{p5_0,  (int)UART_1, 0x03},
		{p5_1,  (int)UART_5, 0x00},
};

const PinMap PinMap_UART_TX[] = {
		{p0_1,  (int)UART_4, 0x12},
		{p0_5,  (int)UART_3, 0x02},
		{p1_5,  (int)UART_1, 0x02},
		{p1_7,  (int)UART_1, 0x02},
		{p2_5,  (int)UART_2, 0x12},
		{p2_14, (int)UART_3, 0x02},
		{p3_5,  (int)UART_6, 0x11},
		{p5_0,  (int)UART_5, 0x01},
		{p5_1,  (int)UART_1, 0x01},
};

const PinMap PinMap_I2C_SCL[] = {
		{p0_8,  (int)I2C_1, 0x12},
		{p0_11, (int)I2C_3, 0x02},
		{p1_1,  (int)I2C_1, 0x02},
		{p2_4,  (int)I2C_2, 0x02},
		{p3_0,  (int)I2C_2, 0x12},
		{p3_6,  (int)I2C_6, 0x11},
		{p5_2,  (int)I2C_5, 0x01},
};

const PinMap PinMap_I2C_SDA[] = {
		{p0_5,  (int)I2C_3, 0x12},
		{p1_5,  (int)I2C_1, 0x02},
		{p2_5,  (int)I2C_2, 0x12},
		{p2_14, (int)I2C_3, 0x32},
		{p3_5,  (int)I2C_6, 0x01},
		{p5_0,  (int)I2C_5, 0x11},
};
