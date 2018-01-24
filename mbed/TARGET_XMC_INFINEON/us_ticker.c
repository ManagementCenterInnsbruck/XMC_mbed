/*
 * us_ticker.c
 *
 *  Created on: 04.04.2017
 *      Author: aamark
 */

#include "us_ticker_api.h"
#include "device.h"
#include "xmc_ccu4.h"

#define US_TICKER_GLOBAL		CCU43
#define US_TICKER_TIMER_1		CCU43_CC40
#define US_TICKER_TIMER_2		CCU43_CC41
#define US_TICKER_TIMER_3		CCU43_CC42
#define US_TICKER_SLICE_NR_1	0
#define US_TICKER_SLICE_NR_2	1
#define US_TICKER_SLICE_NR_3	2
#define US_TICKER_ISR			CCU43_0_IRQn

static bool us_ticker_inited = false;

void us_ticker_concat_irq_handler()
{
	if (XMC_CCU4_SLICE_GetEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP) &&
		XMC_CCU4_SLICE_GetEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP))
	{
		us_ticker_irq_handler();
	}

	XMC_CCU4_SLICE_ClearEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);
	XMC_CCU4_SLICE_ClearEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);
}

void us_ticker_init(void)
{
    if (us_ticker_inited) {
        return;
    }

    XMC_CCU4_Init((XMC_CCU4_MODULE_t*)US_TICKER_GLOBAL, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);

    XMC_CCU4_SLICE_COMPARE_CONFIG_t slice_config;
    memset(&slice_config, 0, sizeof(slice_config));

    XMC_CCU4_SLICE_CompareInit((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_1, &slice_config);
    XMC_CCU4_SLICE_SetTimerPeriodMatch((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_1, CLOCK_FREQ-1);

    slice_config.timer_concatenation = 1;
    XMC_CCU4_SLICE_CompareInit((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, &slice_config);
    XMC_CCU4_SLICE_SetTimerPeriodMatch((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, 0xFFFF);

    XMC_CCU4_SLICE_CompareInit((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, &slice_config);
    XMC_CCU4_SLICE_SetTimerPeriodMatch((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, 0xFFFF);

    XMC_CCU4_EnableShadowTransfer((XMC_CCU4_MODULE_t*)US_TICKER_GLOBAL, (1 << 4 * US_TICKER_SLICE_NR_1) |
    		                                                            (1 << 4 * US_TICKER_SLICE_NR_2) |
																		(1 << 4 * US_TICKER_SLICE_NR_3));

    XMC_CCU4_EnableClock((XMC_CCU4_MODULE_t*)US_TICKER_GLOBAL, US_TICKER_SLICE_NR_1);
    XMC_CCU4_EnableClock((XMC_CCU4_MODULE_t*)US_TICKER_GLOBAL, US_TICKER_SLICE_NR_2);
    XMC_CCU4_EnableClock((XMC_CCU4_MODULE_t*)US_TICKER_GLOBAL, US_TICKER_SLICE_NR_3);

    XMC_CCU4_SLICE_StartTimer((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3);
    XMC_CCU4_SLICE_StartTimer((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2);
    XMC_CCU4_SLICE_StartTimer((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_1);

    us_ticker_inited = true;
}

uint32_t us_ticker_read()
{
	static uint32_t timer_value_prev = 0;

    if (!us_ticker_inited) {
        us_ticker_init();
    }

    uint32_t timer_value = (US_TICKER_TIMER_3->TIMER << 16) | US_TICKER_TIMER_2->TIMER;

    //TimerLSBs is reset before TimerMSBs is incremented
    if (timer_value_prev > timer_value && (timer_value & 0xFFFF) == 0)
    	return timer_value_prev;

    timer_value_prev = timer_value;

    return timer_value;
}

void us_ticker_set_interrupt(timestamp_t timestamp)
{
	XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, timestamp & 0xFFFF);
	XMC_CCU4_SLICE_SetTimerCompareMatch((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, (timestamp >> 16) & 0xFFFF);

	XMC_CCU4_SLICE_StopTimer((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2);
	XMC_CCU4_SLICE_StopTimer((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3);
    XMC_CCU4_EnableShadowTransfer((XMC_CCU4_MODULE_t*)US_TICKER_GLOBAL, (1 << 4 * US_TICKER_SLICE_NR_2) |
																		(1 << 4 * US_TICKER_SLICE_NR_3));
	XMC_CCU4_SLICE_StartTimer((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3);
	XMC_CCU4_SLICE_StartTimer((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2);

	XMC_CCU4_SLICE_SetInterruptNode((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP, XMC_CCU4_SLICE_SR_ID_0);
	XMC_CCU4_SLICE_SetInterruptNode((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP, XMC_CCU4_SLICE_SR_ID_0);
	XMC_CCU4_SLICE_EnableEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);
	XMC_CCU4_SLICE_EnableEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);

	NVIC_SetVector(US_TICKER_ISR, (uint32_t)us_ticker_concat_irq_handler);
	NVIC_EnableIRQ(US_TICKER_ISR);
}

void us_ticker_disable_interrupt(void)
{
	XMC_CCU4_SLICE_DisableEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);
	XMC_CCU4_SLICE_DisableEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);

	NVIC_DisableIRQ(US_TICKER_ISR);
}

void us_ticker_clear_interrupt(void)
{
	XMC_CCU4_SLICE_ClearEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_2, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);
	XMC_CCU4_SLICE_ClearEvent((XMC_CCU4_SLICE_t*)US_TICKER_TIMER_3, XMC_CCU4_SLICE_IRQ_ID_COMPARE_MATCH_UP);

	NVIC_ClearPendingIRQ(US_TICKER_ISR);
}
