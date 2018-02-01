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
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include <cmsis.h>
#include "PinNamesTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PORT_SHIFT  3

typedef enum {

	p0_0  = 0x00,
	p0_1  = 0x01,
	p0_2  = 0x02,
	p0_3  = 0x03,
	p0_4  = 0x04,
	p0_5  = 0x05,
	p0_6  = 0x06,
	p0_7  = 0x07,
	p0_8  = 0x08,
	p0_9  = 0x09,
	p0_10 = 0x0a,
	p0_11 = 0x0b,
	p0_12 = 0x0c,
	p0_13 = 0x0d,
	p0_14 = 0x0e,
	p0_15 = 0x0f,

	p1_0  = 0x10,
	p1_1  = 0x11,
	p1_2  = 0x12,
	p1_3  = 0x13,
	p1_4  = 0x14,
	p1_5  = 0x15,
	p1_6  = 0x16,
	p1_7  = 0x17,
	p1_8  = 0x18,
	p1_9  = 0x19,
	p1_10 = 0x1a,
	p1_11 = 0x1b,
	p1_12 = 0x1c,
	p1_13 = 0x1d,
	p1_14 = 0x1e,
	p1_15 = 0x1f,

	p2_0  = 0x20,
	p2_1  = 0x21,
	p2_2  = 0x22,
	p2_3  = 0x23,
	p2_4  = 0x24,
	p2_5  = 0x25,
	p2_6  = 0x26,
	p2_7  = 0x27,
	p2_8  = 0x28,
	p2_9  = 0x29,
	p2_10 = 0x2a,
	p2_11 = 0x2b,
	p2_12 = 0x2c,
	p2_13 = 0x2d,
	p2_14 = 0x2e,
	p2_15 = 0x2f,

	p3_0  = 0x30,
	p3_1  = 0x31,
	p3_2  = 0x32,
	p3_3  = 0x33,
	p3_4  = 0x34,
	p3_5  = 0x35,
	p3_6  = 0x36,
	p3_7  = 0x37,
	p3_8  = 0x38,
	p3_9  = 0x39,
	p3_10 = 0x3a,
	p3_11 = 0x3b,
	p3_12 = 0x3c,
	p3_13 = 0x3d,
	p3_14 = 0x3e,
	p3_15 = 0x3f,

	p4_0  = 0x40,
	p4_1  = 0x41,
	p4_2  = 0x42,
	p4_3  = 0x43,
	p4_4  = 0x44,
	p4_5  = 0x45,
	p4_6  = 0x46,
	p4_7  = 0x47,

	p5_0  = 0x50,
	p5_1  = 0x51,
	p5_2  = 0x52,
	p5_3  = 0x53,
	p5_4  = 0x54,
	p5_5  = 0x55,
	p5_6  = 0x56,
	p5_7  = 0x57,
	p5_8  = 0x58,
	p5_9  = 0x59,
	p5_10 = 0x5a,
	p5_11 = 0x5b,

	p6_0  = 0x60,
	p6_1  = 0x61,
	p6_2  = 0x62,
	p6_3  = 0x63,
	p6_4  = 0x64,
	p6_5  = 0x65,
	p6_6  = 0x66,

	p14_0  = 0xe0,
	p14_1  = 0xe1,
	p14_2  = 0xe2,
	p14_3  = 0xe3,
	p14_4  = 0xe4,
	p14_5  = 0xe5,
	p14_6  = 0xe6,
	p14_7  = 0xe7,
	p14_8  = 0xe8,
	p14_9  = 0xe9,
	p14_10 = 0xea,
	p14_11 = 0xeb,
	p14_12 = 0xec,
	p14_13 = 0xed,
	p14_14 = 0xee,
	p14_15 = 0xef,

	p15_2  = 0xf2,
	p15_3  = 0xf3,
	p15_4  = 0xf4,
	p15_5  = 0xf5,
	p15_6  = 0xf6,
	p15_7  = 0xf7,
	p15_8  = 0xf8,
	p15_9  = 0xf9,
	p15_10 = 0xfa,
	p15_11 = 0xfb,
	p15_12 = 0xfc,
	p15_13 = 0xfd,
	p15_14 = 0xfe,
	p15_15 = 0xff,

    LED1    = p1_1,
    LED2    = p1_0,

	BUTTON1 = p1_14,
	BUTTON2 = p1_15,

	USBTX 	= (int)0x1FFFFFFF,
	USBRX	= (int)0x2FFFFFFF,

    // Not connected
    NC = (int)0xFFFFFFFF
} PinName;

typedef enum {
    PullNone = 0,
    PullDown = 1,
    PullUp = 3,
    PullDefault = PullUp
} PinMode;

#ifdef __cplusplus
}
#endif

#endif
