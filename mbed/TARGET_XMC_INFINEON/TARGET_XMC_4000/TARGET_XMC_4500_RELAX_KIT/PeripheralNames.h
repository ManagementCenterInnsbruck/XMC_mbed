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
#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include <cmsis.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX
#define STDIO_UART        &VirtualSerial_CDC_Interface

typedef enum {
    UART_1 = 0x40030000UL,
	UART_2 = 0x40030200UL,
	UART_3 = 0x48020000UL,
	UART_4 = 0x48020200UL,
	UART_5 = 0x48024000UL,
	UART_6 = 0x48024200UL,

} UARTName;


typedef enum {
    SPI_0 = 0x40030000UL,
    SPI_1 = 0x40030200UL,
    SPIS = 0x40030000UL
} SPIName;

typedef enum {
    PWM_1  = 0x4000C100UL,
    PWM_2  = 0x4000C200UL,
	PWM_3  = 0x4000C300UL,
	PWM_4  = 0x4000C400UL,
    PWM_5  = 0x40010100UL,
    PWM_6  = 0x40010200UL,
	PWM_7  = 0x40010300UL,
	PWM_8  = 0x40010400UL,
    PWM_9  = 0x40014100UL,
    PWM_10 = 0x40014200UL,
	PWM_11 = 0x40014300UL,
	PWM_12 = 0x40014400UL,
} PWMName;

typedef enum {
    I2C_1 = 0x40030000UL,
	I2C_2 = 0x40030200UL,
	I2C_3 = 0x48020000UL,
	I2C_4 = 0x48020200UL,
	I2C_5 = 0x48024000UL,
	I2C_6 = 0x48024200UL,
} I2CName;

typedef enum {
	DAC_0 = 0x48018000UL,
} DACName;

typedef enum {
    VADC_0 = 0x40004400UL,
	VADC_1 = 0x40004800UL,
	VADC_2 = 0x40004C00UL,
	VADC_3 = 0x40005000UL,
} ADCName;

#ifdef __cplusplus
}
#endif

#endif
