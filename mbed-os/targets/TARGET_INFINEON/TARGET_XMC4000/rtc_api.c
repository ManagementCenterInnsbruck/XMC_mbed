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

#include "device.h"

#if DEVICE_RTC

#include "mbed_mktime.h"
#include "xmc_rtc.h"

/** Initialize the RTC peripheral
 *
 */
void rtc_init(void)
{
	XMC_RTC_CONFIG_t config =
	{
		.prescaler = 0x7fff
	};
	XMC_RTC_Init(&config);
	XMC_RTC_Start();
}

/** Deinitialize RTC
 *
 */
void rtc_free(void)
{
	XMC_RTC_Stop();
}

/** Get the RTC enable status
 *
 * @retval 0 disabled
 * @retval 1 enabled
 */
int rtc_isenabled(void)
{
	if (XMC_RTC_IsEnabled())
		return 1;
	else
		return 0;
}

/** Get the current time from the RTC peripheral
 *
 * @return The current time
 */
time_t rtc_read(void)
{
    struct tm t;

    XMC_RTC_GetTimeStdFormat(&t);
    return mktime(&t);
}

/** Set the current time to the RTC peripheral
 *
 * @param t The current time to be set
 */
void rtc_write(time_t t)
{
	XMC_RTC_SetTimeStdFormat(localtime(&t));
}

#endif /* DEVICE_RTC */
