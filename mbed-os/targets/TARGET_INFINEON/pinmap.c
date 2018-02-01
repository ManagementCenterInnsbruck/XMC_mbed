/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
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
#include "mbed_assert.h"
#include "gpio_api.h"
#include "pinmap.h"
#include "PortNames.h"
#include "mbed_error.h"
#include "xmc_gpio.h"

/**
 * Configure pin (mode, speed, output type and pull-up/pull-down)
 */
void pin_function(PinName pin, int data)
{
    MBED_ASSERT(pin != (PinName)NC);

    gpio_t gpio;
    gpio_init(&gpio, pin);

    XMC_GPIO_SetMode(gpio.port, gpio.pin, (XMC_GPIO_MODE_t)data);
}

/**
 * Configure pin pull-up/pull-down
 */
void pin_mode(PinName pin, PinMode mode)
{
    MBED_ASSERT(pin != (PinName)NC);

    gpio_t gpio;
    gpio_init(&gpio, pin);

    XMC_GPIO_MODE_t xmc_mode;

    switch(mode)
    {
    	case PullNone:
    		xmc_mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    		break;
    	case PullUp:
    		xmc_mode = XMC_GPIO_MODE_INPUT_PULL_UP;
    		break;
    	case PullDown:
    		xmc_mode = XMC_GPIO_MODE_INPUT_PULL_DOWN;
    		break;
    }

    XMC_GPIO_SetMode(gpio.port, gpio.pin, xmc_mode);
}
