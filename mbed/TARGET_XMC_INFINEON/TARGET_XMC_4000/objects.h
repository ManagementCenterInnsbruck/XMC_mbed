/* mbed Microcontroller Library
 * Copyright (c) 2013 Nordic Semiconductor
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

#include <cmsis.h>
#include <PeripheralNames.h>
#include <PinNames.h>
#include <PortNames.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {

} i2c_spi_peripheral_t;

struct serial_s {
	UARTName uart;
	int index;
	PinName rx_pin;
	PinName tx_pin;
    uint32_t baudrate;
    uint32_t databits;
    uint32_t stopbits;
    uint32_t parity;
    uint8_t channel;
    uint8_t usb;
#if DEVICE_SERIAL_ASYNCH
    uint32_t events;
    uint8_t tx_busy;
    uint8_t rx_busy;
#endif
};

struct spi_s {

};

struct port_s {

};

struct pwmout_s {
	PWMName pwm;
	int8_t slice_no;
	int32_t global;
	uint32_t period;
	uint32_t compare;
	uint8_t prescaler;
};

#if DEVICE_I2C_ASYNCH
typedef enum
{
	ASYNCH_Start,
	ASYNCH_Transmit,
	ASYNCH_Receive,
	ASYNCH_Stop,
}asynch_state_t;
#endif

struct i2c_s {
	I2CName i2c;
	int index;
	uint8_t channel;
	PinName sda;
	PinName scl;
    uint8_t repeated_start;
    uint8_t start_condition_byte;
#if DEVICE_I2CSLAVE
    uint8_t slave;
#endif
#if DEVICE_I2C_ASYNCH
    uint32_t address;
    uint8_t stop;
    uint8_t available_events;
    asynch_state_t async_state;
	uint8_t transfer_busy;
#endif
};

struct dac_s {
	DACName dac;
	uint8_t channel_no;
};

struct analogin_s {
	ADCName adc;
	uint8_t group_no;
	uint8_t channel_no;
};

struct gpio_irq_s {
	uint32_t*	eru;
	uint8_t		etl;
	IRQn_Type	irq_n;
	PinName		pin;
};

#include <gpio_object.h>

#ifdef __cplusplus
}
#endif

#endif
