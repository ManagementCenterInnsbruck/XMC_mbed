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
#include "cmsis.h"
#include "gpio_irq_device.h"

// Used to return the index for channels array.
const eru_lines_t pin_lines_desc[] =
{
	{.eru = XMC_ERU0, .channel = 0, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P0.0
	{.eru = XMC_ERU0, .channel = 0, .source = XMC_ERU_ETL_SOURCE_A, .input = 0},			// P0.1
	{.eru = XMC_ERU0, .channel = 3, .source = XMC_ERU_ETL_SOURCE_B, .input = 3},			// P0.2
	{.eru = XMC_ERU1, .channel = 3, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P0.3
	{.eru = XMC_ERU0, .channel = 2, .source = XMC_ERU_ETL_SOURCE_B, .input = 3},			// P0.4
	{.eru = XMC_ERU1, .channel = 3, .source = XMC_ERU_ETL_SOURCE_A, .input = 0},			// P0.5
	{.eru = XMC_ERU0, .channel = 3, .source = XMC_ERU_ETL_SOURCE_B, .input = 2},			// P0.6
	{.eru = XMC_ERU0, .channel = 2, .source = XMC_ERU_ETL_SOURCE_B, .input = 1},			// P0.7
	{.eru = XMC_ERU0, .channel = 2, .source = XMC_ERU_ETL_SOURCE_A, .input = 1},			// P0.8
	{.eru = XMC_ERU0, .channel = 1, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P0.9
	{.eru = XMC_ERU0, .channel = 1, .source = XMC_ERU_ETL_SOURCE_A, .input = 0},			// P0.10
	{.eru = XMC_ERU0, .channel = 3, .source = XMC_ERU_ETL_SOURCE_A, .input = 2},			// P0.11
	{.eru = XMC_ERU0, .channel = 2, .source = XMC_ERU_ETL_SOURCE_B, .input = 2},			// P0.12
	{.eru = XMC_ERU0, .channel = 2, .source = XMC_ERU_ETL_SOURCE_A, .input = 2},			// P0.13
	{0},																					// P0.14
	{0},																					// P0.15
	{.eru = XMC_ERU0, .channel = 3, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P1.0
	{.eru = XMC_ERU0, .channel = 3, .source = XMC_ERU_ETL_SOURCE_A, .input = 0},			// P1.1
	{.eru = XMC_ERU1, .channel = 2, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P1.2
	{.eru = XMC_ERU1, .channel = 2, .source = XMC_ERU_ETL_SOURCE_A, .input = 0},			// P1.3
	{.eru = XMC_ERU0, .channel = 2, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P1.4
	{.eru = XMC_ERU0, .channel = 2, .source = XMC_ERU_ETL_SOURCE_A, .input = 0},			// P1.5
	{0},																					// P1.6
	{0},																					// P1.7
	{0},																					// P1.8
	{0},																					// P1.9
	{0},																					// P1.10
	{0},																					// P1.11
	{0},																					// P1.12
	{0},																					// P1.13
	{0},																					// P1.14
	{.eru = XMC_ERU1, .channel = 1, .source = XMC_ERU_ETL_SOURCE_A, .input = 0},			// P1.15
	{.eru = XMC_ERU0, .channel = 0, .source = XMC_ERU_ETL_SOURCE_B, .input = 3},			// P2.0
	{.eru = XMC_ERU1, .channel = 0, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P2.1
	{.eru = XMC_ERU0, .channel = 1, .source = XMC_ERU_ETL_SOURCE_B, .input = 2},			// P2.2
	{.eru = XMC_ERU0, .channel = 1, .source = XMC_ERU_ETL_SOURCE_A, .input = 2},			// P2.3
	{.eru = XMC_ERU0, .channel = 0, .source = XMC_ERU_ETL_SOURCE_B, .input = 2},			// P2.4
	{.eru = XMC_ERU0, .channel = 0, .source = XMC_ERU_ETL_SOURCE_A, .input = 2},			// P2.5
	{.eru = XMC_ERU0, .channel = 1, .source = XMC_ERU_ETL_SOURCE_B, .input = 3},			// P2.6
	{.eru = XMC_ERU1, .channel = 1, .source = XMC_ERU_ETL_SOURCE_B, .input = 0},			// P2.7
	{0},																					// P2.8
	{0},																					// P2.9
	{0},																					// P2.10
	{0},																					// P2.11
	{0},																					// P2.12
	{0},																					// P2.13
	{0},																					// P2.14
	{0},																					// P2.15
	{0},																					// P3.0
	{.eru = XMC_ERU0, .channel = 0, .source = XMC_ERU_ETL_SOURCE_B, .input = 1},			// P3.1
	{.eru = XMC_ERU0, .channel = 0, .source = XMC_ERU_ETL_SOURCE_A, .input = 1},			// P3.2
	{0},																					// P3.3
	{0},																					// P3.4
	{.eru = XMC_ERU0, .channel = 3, .source = XMC_ERU_ETL_SOURCE_B, .input = 1},			// P3.5
	{.eru = XMC_ERU0, .channel = 3, .source = XMC_ERU_ETL_SOURCE_A, .input = 1},			// P3.6
};

