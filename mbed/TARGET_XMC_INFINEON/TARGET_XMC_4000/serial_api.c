#include "mbed_assert.h"
#include "serial_api.h"

#if DEVICE_SERIAL

#include "pinmap.h"
#include "PeripheralPins.h"
#include "gpio_api.h"
#include "xmc_uart.h"
#include "xmc_gpio.h"
#include "xmc_usbd.h"
#include "VirtualSerial.h"


#define UART_NUM (7)

#define USB_UART_NUM 6

#define FIFO_BUFFER_SIZE 16

static uint32_t serial_irq_ids[UART_NUM] = {0};

static uart_irq_handler irq_handler;

int stdio_uart_inited = 0;
serial_t stdio_uart;

serial_t* usb_uart;

int usb_irq_enabled = 0;

#if DEVICE_SERIAL_ASYNCH
    #define SERIAL_S(obj) (&((obj)->serial))
	int usb_irq_async = 0;
	int usb_async_event = 0;
#else
    #define SERIAL_S(obj) (obj)
#endif

static void usb_irq (uint8_t ep_addr, XMC_USBD_EP_EVENT_t ep_event);

void serial_update_parameter(serial_t *obj)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (obj_s->usb)
		return;

	gpio_t tx;
	gpio_init_port(&tx, obj_s->tx_pin);

	XMC_GPIO_SetMode(tx.port, tx.pin, XMC_GPIO_MODE_INPUT_TRISTATE);

	while(XMC_UART_CH_Stop((XMC_USIC_CH_t*)obj_s->uart) != XMC_UART_CH_STATUS_OK);

	// Init USIC channel
	XMC_UART_CH_CONFIG_t uart_config =
	{
		.baudrate      = obj_s->baudrate,
		.data_bits     = obj_s->databits,
		.frame_length  = obj_s->databits,
		.stop_bits     = obj_s->stopbits,
		.oversampling  = 16U,
		.parity_mode   = obj_s->parity,
	};
	XMC_UART_CH_Init((XMC_USIC_CH_t*)obj_s->uart, &uart_config);

	XMC_UART_CH_Start((XMC_USIC_CH_t*)obj_s->uart);

	XMC_GPIO_SetMode(tx.port, tx.pin, XMC_GPIO_MODE_OUTPUT_PUSH_PULL |
									  (pinmap_function(obj_s->tx_pin, PinMap_UART_TX) & 0x0F) << PORT0_IOCR0_PC0_Pos);
}

void serial_init(serial_t *obj, PinName tx, PinName rx)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (tx == USBTX || rx == USBRX)
		obj_s->usb = 1;
	else
		obj_s->usb = 0;

	if (obj_s->usb)
	{
		USB_runtime.cb_endpoint_event = usb_irq;
		USB_Init();
		obj_s->uart = (UARTName)&VirtualSerial_CDC_Interface;
		obj_s->index = USB_UART_NUM;
		usb_uart = obj;
	}
	else
	{
		UARTName uart_tx = (UARTName)pinmap_find_peripheral(tx, PinMap_UART_TX);
		UARTName uart_rx = (UARTName)pinmap_find_peripheral(rx, PinMap_UART_RX);

		obj_s->uart = (UARTName)pinmap_merge(uart_tx, uart_rx);
		MBED_ASSERT(obj_s->uart != (UARTName)NC);

		obj_s->rx_pin = rx;
		obj_s->tx_pin = tx;

		obj_s->baudrate = 9600;
		obj_s->databits = 8U;
		obj_s->stopbits = 1U;
		obj_s->parity = XMC_USIC_CH_PARITY_MODE_NONE;

		switch (obj_s->uart)
		{
			case UART_1:
				obj_s->index = 0;
				break;
			case UART_2:
				obj_s->index = 1;
				break;
			case UART_3:
				obj_s->index = 2;
				break;
			case UART_4:
				obj_s->index = 3;
				break;
			case UART_5:
				obj_s->index = 4;
				break;
			case UART_6:
				obj_s->index = 5;
				break;
		}

		uint8_t rx_function = pinmap_function(obj_s->rx_pin, PinMap_UART_RX);
		uint8_t tx_function = pinmap_function(obj_s->tx_pin, PinMap_UART_TX);

		// Init Rx-Pin
		XMC_GPIO_CONFIG_t rx_pin_config =
		{
			.mode 			= XMC_GPIO_MODE_INPUT_TRISTATE,
			.output_level 	= XMC_GPIO_OUTPUT_LEVEL_HIGH,
		};

		gpio_t rx_gpio;
		gpio_init_port(&rx_gpio, obj_s->rx_pin);

		XMC_GPIO_Init(rx_gpio.port, rx_gpio.pin, &rx_pin_config);

		// Init USIC channel
		XMC_UART_CH_CONFIG_t uart_config =
		{
			.baudrate      = obj_s->baudrate,
			.data_bits     = obj_s->databits,
			.frame_length  = obj_s->databits,
			.stop_bits     = obj_s->stopbits,
			.oversampling  = 16U,
			.parity_mode   = obj_s->parity,
		};
		XMC_UART_CH_Init((XMC_USIC_CH_t*)obj_s->uart, &uart_config);

		// Input source path
		uint8_t source = rx_function & 0x0F;
		XMC_USIC_CH_SetInputSource((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_INPUT_DX0, source);

		// Configuration FIFO
		obj_s->channel = (rx_function & 0xF0) >> 4;
		XMC_USIC_CH_TXFIFO_Configure((XMC_USIC_CH_t*)obj_s->uart, 0 + (32 * obj_s->channel), XMC_USIC_CH_FIFO_SIZE_16WORDS, 1);
		XMC_USIC_CH_RXFIFO_Configure((XMC_USIC_CH_t*)obj_s->uart, 16 + (32 * obj_s->channel), XMC_USIC_CH_FIFO_SIZE_16WORDS, 0);

		XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 0 + (2*obj_s->channel));
		XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 1 + (2*obj_s->channel));
		XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 1 + (2*obj_s->channel));

		XMC_UART_CH_Start((XMC_USIC_CH_t*)obj_s->uart);

		//Init Tx-Pin
		XMC_GPIO_CONFIG_t tx_pin_config =
		{
			.mode			= XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ((tx_function & 0x0F) << PORT0_IOCR0_PC0_Pos),
			.output_level	= XMC_GPIO_OUTPUT_LEVEL_HIGH,
		};

		gpio_t tx_gpio;
		gpio_init_port(&tx_gpio, obj_s->tx_pin);

		XMC_GPIO_Init(tx_gpio.port, tx_gpio.pin, &tx_pin_config);

		if (obj_s->uart == STDIO_UART)
		{
			stdio_uart_inited = 1;
			memcpy(&stdio_uart, obj, sizeof(serial_t));
		}
	}
}

void serial_putc(serial_t *obj, int c)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (obj_s->usb)
	{
		CDC_Device_SendByte(&VirtualSerial_CDC_Interface, c);
		CDC_Device_Flush(&VirtualSerial_CDC_Interface);
		while(!Endpoint_IsINReady());
	}
	else
	{
		while (XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->uart));

		XMC_UART_CH_Transmit((XMC_USIC_CH_t*)obj_s->uart, c);
	}
}

int serial_getc(serial_t *obj)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (obj_s->usb)
	{
		while (!CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface));

		return CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	}
	else
	{
		while (!XMC_USIC_CH_RXFIFO_GetLevel((XMC_USIC_CH_t*)obj_s->uart));

		return XMC_UART_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->uart);
	}
}

void serial_baud(serial_t *obj, int baudrate)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (obj_s->usb)
		return;

	obj_s->baudrate = baudrate;
	serial_update_parameter(obj);
}

void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (obj_s->usb)
		return;

	obj_s->databits = data_bits;
	obj_s->stopbits = stop_bits;

	switch (parity)
	{
		case ParityEven:
			obj_s->parity = XMC_USIC_CH_PARITY_MODE_EVEN;
			break;
		case ParityOdd:
			obj_s->parity = XMC_USIC_CH_PARITY_MODE_ODD;
			break;
		default:
			obj_s->parity = XMC_USIC_CH_PARITY_MODE_NONE;
			break;
	}

	serial_update_parameter(obj);
}

int serial_readable(serial_t *obj)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	uint8_t level;

	if (obj_s->usb)
		level = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);
	else
		level = XMC_USIC_CH_RXFIFO_GetLevel((XMC_USIC_CH_t*)obj_s->uart);

	if (level)
		return 1;
	else
		return 0;
}

int serial_writable(serial_t *obj)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (obj_s->usb)
		return 1;

	if (XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->uart))
		return 0;
	else
		return 1;
}

void serial_break_set(serial_t *obj)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	if (obj_s->usb)
		return;

	/* Check FIFO size */
	if ((((XMC_USIC_CH_t*)obj_s->uart)->TBCTR & USIC_CH_TBCTR_SIZE_Msk) == 0)
		((XMC_USIC_CH_t*)obj_s->uart)->TBUF[11] = 0;
	else
		((XMC_USIC_CH_t*)obj_s->uart)->IN[11] = 0;
}

void serial_break_clear(serial_t *obj)
{
	(void)obj;
}

void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	irq_handler = handler;
	serial_irq_ids[obj_s->index] = id;
}

static void uart_rx_irq(XMC_USIC_CH_t* channel, int id)
{
	uint32_t event = XMC_USIC_CH_RXFIFO_GetEvent(channel);

	if (event == XMC_USIC_CH_RXFIFO_EVENT_STANDARD || event == XMC_USIC_CH_RXFIFO_EVENT_ALTERNATE)
	{
		irq_handler(serial_irq_ids[id], RxIrq);
		XMC_USIC_CH_RXFIFO_ClearEvent(channel, event);
	}
}

static void uart_tx_irq(XMC_USIC_CH_t* channel, int id)
{
	if (XMC_USIC_CH_TXFIFO_GetEvent(channel) == XMC_USIC_CH_TXFIFO_EVENT_STANDARD)
	{
		irq_handler(serial_irq_ids[id], TxIrq);
		XMC_USIC_CH_TXFIFO_ClearEvent(channel, XMC_USIC_CH_TXFIFO_EVENT_STANDARD);
	}
}

static void uart1_rx_irq()
{
	uart_rx_irq((XMC_USIC_CH_t*)USIC0_CH0, 0);
}

static void uart1_tx_irq()
{
	uart_tx_irq((XMC_USIC_CH_t*)USIC0_CH0, 0);
}

static void uart2_rx_irq()
{
	uart_rx_irq((XMC_USIC_CH_t*)USIC0_CH1, 1);
}

static void uart2_tx_irq()
{
	uart_tx_irq((XMC_USIC_CH_t*)USIC0_CH1, 1);
}

static void uart3_rx_irq()
{
	uart_rx_irq((XMC_USIC_CH_t*)USIC1_CH0, 2);
}

static void uart3_tx_irq()
{
	uart_tx_irq((XMC_USIC_CH_t*)USIC1_CH0, 2);
}

static void uart4_rx_irq()
{
	uart_rx_irq((XMC_USIC_CH_t*)USIC1_CH1, 3);
}

static void uart4_tx_irq()
{
	uart_tx_irq((XMC_USIC_CH_t*)USIC1_CH1, 3);
}

static void uart5_rx_irq()
{
	uart_rx_irq((XMC_USIC_CH_t*)USIC2_CH0, 4);
}

static void uart5_tx_irq()
{
	uart_tx_irq((XMC_USIC_CH_t*)USIC2_CH0, 4);
}

static void uart6_rx_irq()
{
	uart_rx_irq((XMC_USIC_CH_t*)USIC2_CH1, 5);
}

static void uart6_tx_irq()
{
	uart_tx_irq((XMC_USIC_CH_t*)USIC2_CH1, 5);
}

static void usb_irq (uint8_t ep_addr, XMC_USBD_EP_EVENT_t ep_event)
{
	XMC_USBD_EVENT_OUT_EP_t in_event;
	XMC_USBD_EVENT_IN_EP_t out_event;

	uint8_t ep_num = ep_addr & 0x0f;
	XMC_USBD_EP_t* ep =  &xmc_device.ep[ep_num];

	USBD_SignalEndpointEvent_Handler(ep_addr, ep_event);

	if (usb_irq_enabled && ep_num)
	{
		switch (ep_event)
		{
			case XMC_USBD_EP_EVENT_OUT:
				out_event = xmc_device.endpoint_out_register[ep_num]->doepint & xmc_device.device_register->doepmsk;
				if (out_event & XMC_USBD_EVENT_IN_EP_TX_COMPLET)
					irq_handler(serial_irq_ids[USB_UART_NUM], RxIrq);
				break;
			case XMC_USBD_EP_EVENT_IN:
				in_event = xmc_device.endpoint_in_register[ep_num]->diepint &
				       ((((xmc_device.device_register->dtknqr4_fifoemptymsk >> ep->address_u.address_st.number) & 0x1U) << 7U) | xmc_device.device_register->diepmsk);

				if (in_event & XMC_USBD_EVENT_OUT_EP_TX_COMPLET)
					irq_handler(serial_irq_ids[USB_UART_NUM], TxIrq);
				break;
			default:
				break;
		}
	}

# if DEVICE_SERIAL_ASYNCH

	usb_async_event = 0;

	if (usb_irq_async && ep_num)
	{
		switch (ep_event)
		{
			case XMC_USBD_EP_EVENT_OUT:
				out_event = xmc_device.endpoint_out_register[ep_num]->doepint & xmc_device.device_register->doepmsk;
				if (out_event & XMC_USBD_EVENT_IN_EP_TX_COMPLET)
				{
					uint8_t bytes = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);

					if (usb_uart->rx_buff.width == 16)
					{
						for (uint8_t i=0; i<bytes; i+=2)
						{
							uint8_t msbs = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
							uint8_t lsbs = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

							if (usb_uart->rx_buff.pos < usb_uart->rx_buff.length)
								((uint16_t*)usb_uart->rx_buff.buffer)[usb_uart->rx_buff.pos++] = (msbs <<8) | lsbs;
						}
					}
					else
					{
						for (uint8_t i=0; i<bytes; i++)
						{
							uint8_t rec = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
							if (usb_uart->rx_buff.pos < usb_uart->rx_buff.length)
								((uint8_t*)usb_uart->rx_buff.buffer)[usb_uart->rx_buff.pos++] = rec;
						}
					}

					if (usb_uart->rx_buff.pos == usb_uart->rx_buff.length)
					{
						usb_async_event = SERIAL_EVENT_RX_COMPLETE;
						usb_uart->serial.rx_busy = 0;
						usb_irq_async = 0;
					}
				}
				break;
			case XMC_USBD_EP_EVENT_IN:
				in_event = xmc_device.endpoint_in_register[ep_num]->diepint &
					   ((((xmc_device.device_register->dtknqr4_fifoemptymsk >> ep->address_u.address_st.number) & 0x1U) << 7U) | xmc_device.device_register->diepmsk);

				if (in_event & XMC_USBD_EVENT_OUT_EP_TX_COMPLET)
				{
					usb_async_event = SERIAL_EVENT_TX_COMPLETE;
			    	usb_uart->serial.tx_busy = 0;
					usb_irq_async = 0;
				}
				break;
			default:
				break;
		}
	}
#endif
}

void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
{
	struct serial_s *obj_s = SERIAL_S(obj);

	IRQn_Type irq_n = (IRQn_Type)0;
	uint32_t vector = 0;

	if (obj_s->usb)
		usb_irq_enabled = enable;
	else
	{
		switch (obj_s->uart)
		{
			case UART_1: if (irq == TxIrq) {irq_n = USIC0_0_IRQn; vector = (uint32_t)&uart1_tx_irq;}
						 else              {irq_n = USIC0_1_IRQn; vector = (uint32_t)&uart1_rx_irq;} break;
			case UART_2: if (irq == TxIrq) {irq_n = USIC0_2_IRQn; vector = (uint32_t)&uart2_tx_irq;}
						 else              {irq_n = USIC0_3_IRQn; vector = (uint32_t)&uart2_rx_irq;} break;
			case UART_3: if (irq == TxIrq) {irq_n = USIC1_0_IRQn; vector = (uint32_t)&uart3_tx_irq;}
						 else              {irq_n = USIC1_1_IRQn; vector = (uint32_t)&uart3_rx_irq;} break;
			case UART_4: if (irq == TxIrq) {irq_n = USIC1_2_IRQn; vector = (uint32_t)&uart4_tx_irq;}
						 else              {irq_n = USIC1_3_IRQn; vector = (uint32_t)&uart4_rx_irq;} break;
			case UART_5: if (irq == TxIrq) {irq_n = USIC2_0_IRQn; vector = (uint32_t)&uart5_tx_irq;}
						 else              {irq_n = USIC2_1_IRQn; vector = (uint32_t)&uart5_rx_irq;} break;
			case UART_6: if (irq == TxIrq) {irq_n = USIC2_2_IRQn; vector = (uint32_t)&uart6_tx_irq;}
						 else              {irq_n = USIC2_3_IRQn; vector = (uint32_t)&uart6_rx_irq;} break;
		}

		if (enable)
		{
			if (irq == TxIrq)
				XMC_USIC_CH_TXFIFO_EnableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
			else
				XMC_USIC_CH_RXFIFO_EnableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);

			NVIC_SetVector(irq_n, vector);
			NVIC_EnableIRQ(irq_n);
		}
		else
		{
			if (irq == TxIrq)
				XMC_USIC_CH_TXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
			else
				XMC_USIC_CH_RXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);

			NVIC_DisableIRQ(irq_n);
		}
	}
}

#if DEVICE_SERIAL_ASYNCH

/******************************************************************************
 * LOCAL HELPER FUNCTIONS
 ******************************************************************************/

/**
 * Configure the TX buffer for an asynchronous write serial transaction
 *
 * @param obj       The serial object.
 * @param tx        The buffer for sending.
 * @param tx_length The number of words to transmit.
 */
static void serial_tx_buffer_set(serial_t *obj, void *tx, int tx_length, uint8_t width)
{
    (void)width;

    // Exit if a transmit is already on-going
    if (serial_tx_active(obj)) {
        return;
    }

    obj->tx_buff.buffer = tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos = 0;
    obj->tx_buff.width = width;
}

/**
 * Configure the RX buffer for an asynchronous write serial transaction
 *
 * @param obj       The serial object.
 * @param tx        The buffer for sending.
 * @param tx_length The number of words to transmit.
 */
static void serial_rx_buffer_set(serial_t *obj, void *rx, int rx_length, uint8_t width)
{
    (void)width;

    // Exit if a reception is already on-going
    if (serial_rx_active(obj)) {
        return;
    }

    obj->rx_buff.buffer = rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos = 0;
    obj->rx_buff.width = width;
}

/**
 * Configure events
 *
 * @param obj    The serial object
 * @param event  The logical OR of the events to configure
 * @param enable Set to non-zero to enable events, or zero to disable them
 */
static void serial_enable_event(serial_t *obj, int event, uint8_t enable)
{
    struct serial_s *obj_s = SERIAL_S(obj);

    // Shouldn't have to enable interrupt here, just need to keep track of the requested events.
    if (enable) {
        obj_s->events |= event;
    } else {
        obj_s->events &= ~event;
    }
}

/**
* Get index of serial object TX IRQ, relating it to the physical peripheral.
*
* @param obj pointer to serial object
* @return internal NVIC TX IRQ index of U(S)ART peripheral
*/
static IRQn_Type serial_get_tx_irq_n(serial_t *obj)
{
    struct serial_s *obj_s = SERIAL_S(obj);
    IRQn_Type irq_n;

    switch (obj_s->index) {
        case 0:
            irq_n = USIC0_0_IRQn;
            break;
        case 1:
            irq_n = USIC0_2_IRQn;
            break;
        case 2:
            irq_n = USIC1_0_IRQn;
            break;
        case 3:
            irq_n = USIC1_2_IRQn;
            break;
        case 4:
            irq_n = USIC2_0_IRQn;
            break;
        case 5:
            irq_n = USIC2_2_IRQn;
            break;
        case USB_UART_NUM:
        	irq_n = USB0_0_IRQn;
        	break;
        default:
            irq_n = (IRQn_Type)0;
    }

    return irq_n;
}

/**
* Get index of serial object RX IRQ, relating it to the physical peripheral.
*
* @param obj pointer to serial object
* @return internal NVIC RX IRQ index of U(S)ART peripheral
*/
static IRQn_Type serial_get_rx_irq_n(serial_t *obj)
{
    struct serial_s *obj_s = SERIAL_S(obj);
    IRQn_Type irq_n;

    switch (obj_s->index) {
        case 0:
            irq_n = USIC0_1_IRQn;
            break;
        case 1:
            irq_n = USIC0_3_IRQn;
            break;
        case 2:
            irq_n = USIC1_1_IRQn;
            break;
        case 3:
            irq_n = USIC1_3_IRQn;
            break;
        case 4:
            irq_n = USIC2_1_IRQn;
            break;
        case 5:
            irq_n = USIC2_3_IRQn;
            break;
        case USB_UART_NUM:
        	irq_n = USB0_0_IRQn;
        	break;
        default:
            irq_n = (IRQn_Type)0;
    }

    return irq_n;
}

/******************************************************************************
 * MBED API FUNCTIONS
 ******************************************************************************/

/**
 * Begin asynchronous TX transfer. The used buffer is specified in the serial
 * object, tx_buff
 *
 * @param obj       The serial object
 * @param tx        The buffer for sending
 * @param tx_length The number of words to transmit
 * @param tx_width  The bit width of buffer word
 * @param handler   The serial handler
 * @param event     The logical OR of events to be registered
 * @param hint      A suggestion for how to use DMA with this transfer
 * @return Returns number of data transfered, or 0 otherwise
 */
int serial_tx_asynch(serial_t *obj, const void *tx, size_t tx_length, uint8_t tx_width, uint32_t handler, uint32_t event, DMAUsage hint)
{
    // TODO: DMA usage is currently ignored
    (void) hint;

	MBED_ASSERT(tx != (void*)0);

	struct serial_s *obj_s = SERIAL_S(obj);

    if (tx_length == 0)
        return 0;

    // Set up buffer
    serial_tx_buffer_set(obj, (void *)tx, tx_length, tx_width);

    // Set up events
    serial_enable_event(obj, SERIAL_EVENT_TX_ALL, 0); // Clear all events
    serial_enable_event(obj, event, 1); // Set only the wanted events

	// Enable interrupt
	IRQn_Type irq_n = serial_get_tx_irq_n(obj);
	NVIC_ClearPendingIRQ(irq_n);
	NVIC_DisableIRQ(irq_n);
	NVIC_SetVector(irq_n, (uint32_t)handler);
	NVIC_EnableIRQ(irq_n);

	obj_s->tx_busy = 1;

    if (obj_s->usb)
    {
    	usb_irq_async = 1;
    	obj->tx_buff.pos+=tx_length;
    	if (tx_width == 8)
    		CDC_Device_SendData(&VirtualSerial_CDC_Interface, tx, tx_length);
    	else
    		CDC_Device_SendData(&VirtualSerial_CDC_Interface, tx, 2*tx_length);
		CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    }
    else
    {
		XMC_USIC_CH_TXFIFO_Flush((XMC_USIC_CH_t*)obj_s->uart);

		XMC_USIC_CH_TXFIFO_EnableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
		XMC_USIC_CH_TriggerServiceRequest((XMC_USIC_CH_t*)obj_s->uart, 0 + (2*obj_s->channel));
    }

    return tx_length;
}

/** Begin asynchronous RX transfer (enable interrupt for data collecting)
 *  The used buffer is specified in the serial object - rx_buff
 *
 * @param obj        The serial object
 * @param rx         The receive buffer
 * @param rx_length  The number of bytes to receive
 * @param rx_width   Deprecated argument
 * @param handler    The serial handler
 * @param event      The logical OR of events to be registered
 * @param handler    The serial handler
 * @param char_match A character in range 0-254 to be matched
 * @param hint       A suggestion for how to use DMA with this transfer
 */
void serial_rx_asynch(serial_t *obj, void *rx, size_t rx_length, uint8_t rx_width, uint32_t handler, uint32_t event, uint8_t char_match, DMAUsage hint)
{
    // TODO: DMA usage is currently ignored
    (void) hint;

    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT(rx != (void*)0);

    struct serial_s *obj_s = SERIAL_S(obj);

    serial_enable_event(obj, SERIAL_EVENT_RX_ALL, 0);
    serial_enable_event(obj, event, 1);

    // set CharMatch
    obj->char_match = char_match;

    serial_rx_buffer_set(obj, rx, rx_length, rx_width);

    IRQn_Type irq_n = serial_get_rx_irq_n(obj);
    NVIC_ClearPendingIRQ(irq_n);
    NVIC_DisableIRQ(irq_n);
    NVIC_SetVector(irq_n, (uint32_t)handler);
    NVIC_EnableIRQ(irq_n);

    obj_s->rx_busy = 1;

    if (obj_s->usb)
    {
    	usb_irq_async = 1;
    }
    else
    {
		XMC_USIC_CH_RXFIFO_Flush((XMC_USIC_CH_t*)obj_s->uart);

		if (obj->rx_buff.width == 16)
			rx_length = rx_length << 1;

		if (rx_length > FIFO_BUFFER_SIZE)
			XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_FIFO_SIZE_16WORDS, FIFO_BUFFER_SIZE - 1);
		else
			XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_FIFO_SIZE_16WORDS, rx_length - 1);

		XMC_USIC_CH_RXFIFO_EnableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);
    }
}

/**
 * Attempts to determine if the serial peripheral is already in use for TX
 *
 * @param obj The serial object
 * @return Non-zero if the TX transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
	MBED_ASSERT(obj);

	struct serial_s *obj_s = SERIAL_S(obj);

	return obj_s->tx_busy;
}

/**
 * Attempts to determine if the serial peripheral is already in use for RX
 *
 * @param obj The serial object
 * @return Non-zero if the RX transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
    MBED_ASSERT(obj);

    struct serial_s *obj_s = SERIAL_S(obj);

    return obj_s->rx_busy;
}

/**
 * The asynchronous TX and RX handler.
 *
 * @param obj The serial object
 * @return Returns event flags if a TX/RX transfer termination condition was met or 0 otherwise
 */
int serial_irq_handler_asynch(serial_t *obj)
{
    struct serial_s *obj_s = SERIAL_S(obj);
    volatile int return_event = 0;

    //Virtual USB
    if (obj_s->usb)
    {
    	XMC_USBD_IRQHandler(&USB_runtime);
    	return_event = usb_async_event;
    	usb_async_event = 0;
    }
    else //UART
    {
		// TX PART:
		if (obj_s->tx_busy)
		{
			if (obj->tx_buff.pos == obj->tx_buff.length)
			{
				XMC_USIC_CH_TXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
				obj_s->tx_busy = 0;

				if ((obj_s->events & SERIAL_EVENT_TX_COMPLETE ) != 0)
					return_event |= (SERIAL_EVENT_TX_COMPLETE & obj_s->events);
			}

			while (!XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->uart) &&
					obj->tx_buff.pos < obj->tx_buff.length)
			{
				if (obj->tx_buff.width == 16)
				{
					XMC_UART_CH_Transmit((XMC_USIC_CH_t*)obj_s->uart, ((uint8_t*)obj->tx_buff.buffer)[2*obj->tx_buff.pos+1]);
					XMC_UART_CH_Transmit((XMC_USIC_CH_t*)obj_s->uart, ((uint8_t*)obj->tx_buff.buffer)[2*obj->tx_buff.pos]);
					obj->tx_buff.pos++;
				}
				else
					XMC_UART_CH_Transmit((XMC_USIC_CH_t*)obj_s->uart, ((uint8_t*)obj->tx_buff.buffer)[obj->tx_buff.pos++]);
			}

			XMC_USIC_CH_TXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_EVENT_STANDARD);
		}

		//RX PART
		if (obj_s->rx_busy)
		{
			while (!XMC_USIC_CH_RXFIFO_IsEmpty((XMC_USIC_CH_t*)obj_s->uart))
			{
				if (obj->rx_buff.width == 16)
				{
					uint8_t msbs = XMC_UART_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->uart);
					uint8_t lsbs = XMC_UART_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->uart);

					if (obj->rx_buff.pos < obj->rx_buff.length)
						((uint16_t*)obj->rx_buff.buffer)[obj->rx_buff.pos++] = (msbs << 8) | lsbs;
				}
				else
				{
					uint8_t rec = XMC_UART_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->uart);

					if (obj->rx_buff.pos < obj->rx_buff.length)
						((uint8_t*)obj->rx_buff.buffer)[obj->rx_buff.pos++] = rec;
				}
			}

			if (obj->rx_buff.pos == obj->rx_buff.length)
			{
				XMC_USIC_CH_RXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);
				obj_s->rx_busy = 0;

				return_event |= (SERIAL_EVENT_RX_COMPLETE & obj_s->events);
			}
			else
			{
				uint16_t level = obj->rx_buff.length - obj->rx_buff.pos;

				if (obj->rx_buff.width == 16)
					level = level << 1;

				if (level > FIFO_BUFFER_SIZE)
					XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_FIFO_SIZE_16WORDS, FIFO_BUFFER_SIZE-1);
				else
					XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_FIFO_SIZE_16WORDS, level-1);
			}

			XMC_USIC_CH_RXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_EVENT_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_ALTERNATE);
		}
    }

    //Check if char_match is present
	if (obj_s->events & SERIAL_EVENT_RX_CHARACTER_MATCH)
	{
		uint8_t *buf = (uint8_t*)(obj->rx_buff.buffer);

		if (buf != NULL)
		{
			uint8_t length;

			if (obj->rx_buff.width == 16)
				length = obj->rx_buff.pos << 1;
			else
				length = obj->rx_buff.pos;

			for (uint8_t i = 0; i < length; i++) {
				if (buf[i] == obj->char_match)
				{
					if (obj->rx_buff.width == 16)
						obj->rx_buff.pos = i>>1;
					else
						obj->rx_buff.pos = i;
					return_event |= (SERIAL_EVENT_RX_CHARACTER_MATCH & obj_s->events);
					serial_rx_abort_asynch(obj);
					break;
				}
			}
		}
	}

    return return_event;
}

/**
 * Abort the ongoing TX transaction. It disables the enabled interupt for TX and
 * flush TX hardware buffer if TX FIFO is used
 *
 * @param obj The serial object
 */
void serial_tx_abort_asynch(serial_t *obj)
{
    struct serial_s *obj_s = SERIAL_S(obj);

    XMC_USIC_CH_TXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
    XMC_USIC_CH_TXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_TXFIFO_EVENT_STANDARD);

    XMC_USIC_CH_TXFIFO_Flush((XMC_USIC_CH_t*)obj_s->uart);

    obj_s->tx_busy = 0;
}

/**
 * Abort the ongoing RX transaction It disables the enabled interrupt for RX and
 * flush RX hardware buffer if RX FIFO is used
 *
 * @param obj The serial object
 */
void serial_rx_abort_asynch(serial_t *obj)
{
    struct serial_s *obj_s = SERIAL_S(obj);

	XMC_USIC_CH_RXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);
	XMC_USIC_CH_RXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->uart, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);

	XMC_USIC_CH_RXFIFO_Flush((XMC_USIC_CH_t*)obj_s->uart);

	obj_s->rx_busy = 0;
}

#endif
#endif
