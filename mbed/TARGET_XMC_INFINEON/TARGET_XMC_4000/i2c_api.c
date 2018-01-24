#include "mbed_assert.h"
#include "i2c_api.h"

#if DEVICE_I2C

#include "pinmap.h"
#include "PeripheralPins.h"
#include "xmc_i2c.h"

#if DEVICE_I2C_ASYNCH
    #define I2C_S(obj) (struct i2c_s *) (&((obj)->i2c))
#else
    #define I2C_S(obj) (struct i2c_s *) (obj)
#endif

#define FLAG_TIMEOUT ((int)0x1000)
#define HALF_FIFO_SIZE 8

void i2c_init(i2c_t *obj, PinName sda, PinName scl)
{
	struct i2c_s* obj_s = I2C_S(obj);

	I2CName i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
	I2CName i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);
	obj_s->sda = sda;
	obj_s->scl = scl;

	obj_s->i2c = (I2CName)pinmap_merge(i2c_sda, i2c_scl);
	MBED_ASSERT(obj_s->i2c != (I2CName)NC);

	uint8_t sda_source = (pinmap_function(sda, PinMap_I2C_SDA) & 0xF0) >> 4;
	uint8_t scl_source = (pinmap_function(scl, PinMap_I2C_SCL) & 0xF0) >> 4;

	XMC_I2C_CH_CONFIG_t config =
	{
		.baudrate = 100000,
		.address = 0,
	};

	XMC_I2C_CH_Init((XMC_USIC_CH_t*)obj_s->i2c, &config);
	XMC_USIC_CH_SetInputSource((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_INPUT_SDA, sda_source);
	XMC_USIC_CH_SetInputSource((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_INPUT_SCL, scl_source);

	// Configuration FIFO
	switch (obj_s->i2c)
	{
		case I2C_1:
			obj_s->index = 0;
			obj_s->channel = 0;
			break;
		case I2C_2:
			obj_s->index = 1;
			obj_s->channel = 1;
			break;
		case I2C_3:
			obj_s->index = 2;
			obj_s->channel = 0;
			break;
		case I2C_4:
			obj_s->index = 3;
			obj_s->channel = 1;
			break;
		case I2C_5:
			obj_s->index = 4;
			obj_s->channel = 0;
			break;
		case I2C_6:
			obj_s->index = 5;
			obj_s->channel = 1;
			break;
	}

	XMC_USIC_CH_TXFIFO_Configure((XMC_USIC_CH_t*)obj_s->i2c, 0 + (32 * obj_s->channel), XMC_USIC_CH_FIFO_SIZE_16WORDS, 1);
	XMC_USIC_CH_RXFIFO_Configure((XMC_USIC_CH_t*)obj_s->i2c, 16 + (32 * obj_s->channel), XMC_USIC_CH_FIFO_SIZE_16WORDS, 0);

	XMC_USIC_CH_TXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, obj_s->channel);
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, obj_s->channel);
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, obj_s->channel);
	XMC_I2C_CH_SetInterruptNodePointer((XMC_USIC_CH_t*)obj_s->i2c, obj_s->channel);

	XMC_I2C_CH_Start((XMC_USIC_CH_t*)obj_s->i2c);

	XMC_GPIO_CONFIG_t scl_pin_config =
	{
		.mode			= XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN | ((pinmap_function(scl, PinMap_I2C_SCL) & 0x0F) << PORT0_IOCR0_PC0_Pos),
		.output_level	= XMC_GPIO_OUTPUT_LEVEL_HIGH,
		.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE
	};

	XMC_GPIO_CONFIG_t sda_pin_config =
	{
		.mode			= XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN | ((pinmap_function(sda, PinMap_I2C_SDA) & 0x0F) << PORT0_IOCR0_PC0_Pos),
		.output_level	= XMC_GPIO_OUTPUT_LEVEL_HIGH,
		.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE
	};

	gpio_t sda_gpio, scl_gpio;
	gpio_init_port(&sda_gpio, obj_s->sda);
	gpio_init_port(&scl_gpio, obj_s->scl);

	XMC_GPIO_Init(sda_gpio.port, sda_gpio.pin, &sda_pin_config);
	XMC_GPIO_Init(scl_gpio.port, scl_gpio.pin, &scl_pin_config);
}

int i2c_start(i2c_t *obj)
{
	struct i2c_s* obj_s = I2C_S(obj);

	obj_s->start_condition_byte = 1;

	return 1;
}

int i2c_stop(i2c_t *obj)
{
	struct i2c_s* obj_s = I2C_S(obj);

	while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
	XMC_I2C_CH_MasterStop((XMC_USIC_CH_t*)obj_s->i2c);

	return 1;
}

void i2c_frequency(i2c_t *obj, int hz)
{
	struct i2c_s* obj_s = I2C_S(obj);

	XMC_I2C_CH_SetBaudrate((XMC_USIC_CH_t*)obj_s->i2c, hz);
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
	struct i2c_s* obj_s = I2C_S(obj);

	while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));

	if (obj_s->repeated_start)
		XMC_I2C_CH_MasterRepeatedStart((XMC_USIC_CH_t*)obj_s->i2c, address, XMC_I2C_CH_CMD_WRITE);
	else
		XMC_I2C_CH_MasterStart((XMC_USIC_CH_t*)obj_s->i2c, address, XMC_I2C_CH_CMD_WRITE);

	for (uint32_t i=0; i<length; i++)
	{
		while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
		XMC_I2C_CH_MasterTransmit((XMC_USIC_CH_t*)obj_s->i2c, data[i]);
	}

	if (stop)
	{
		while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
		XMC_I2C_CH_MasterStop((XMC_USIC_CH_t*)obj_s->i2c);
		obj_s->repeated_start = 0;
	}
	else
		obj_s->repeated_start = 1;

	while(!XMC_USIC_CH_TXFIFO_IsEmpty((XMC_USIC_CH_t*)obj_s->i2c));

	return length;
}

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop)
{
	struct i2c_s* obj_s = I2C_S(obj);

	while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));

	if (obj_s->repeated_start)
		XMC_I2C_CH_MasterRepeatedStart((XMC_USIC_CH_t*)obj_s->i2c, address, XMC_I2C_CH_CMD_READ);
	else
		XMC_I2C_CH_MasterStart((XMC_USIC_CH_t*)obj_s->i2c, address, XMC_I2C_CH_CMD_READ);

	if (length > 1)
	{
		XMC_I2C_CH_MasterReceiveAck((XMC_USIC_CH_t*)obj_s->i2c);

		for (uint32_t i=1; i<length; i++)
		{
			while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));

			if (i < length-1)
				XMC_I2C_CH_MasterReceiveAck((XMC_USIC_CH_t*)obj_s->i2c);
			else
				XMC_I2C_CH_MasterReceiveNack((XMC_USIC_CH_t*)obj_s->i2c);

			while(XMC_USIC_CH_RXFIFO_IsEmpty((XMC_USIC_CH_t*)obj_s->i2c));
			data[i-1] = XMC_I2C_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->i2c);
		}
	}
	else
	{
		XMC_I2C_CH_MasterReceiveNack((XMC_USIC_CH_t*)obj_s->i2c);
	}

	if (stop)
	{
		while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
		XMC_I2C_CH_MasterStop((XMC_USIC_CH_t*)obj_s->i2c);
		obj_s->repeated_start = 0;
	}
	else
		obj_s->repeated_start = 1;

	while(XMC_USIC_CH_RXFIFO_IsEmpty((XMC_USIC_CH_t*)obj_s->i2c));
	data[length-1] = XMC_I2C_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->i2c);


	while(!XMC_USIC_CH_TXFIFO_IsEmpty((XMC_USIC_CH_t*)obj_s->i2c));

	return length;
}
void i2c_reset(i2c_t *obj)
{
    struct i2c_s *obj_s = I2C_S(obj);

    i2c_stop(obj);
    i2c_init(obj, obj_s->sda, obj_s->scl);
}

int i2c_byte_read(i2c_t *obj, int last)
{
	struct i2c_s *obj_s = I2C_S(obj);

	while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));

	if (last)
		XMC_I2C_CH_MasterReceiveNack((XMC_USIC_CH_t*)obj_s->i2c);
	else
		XMC_I2C_CH_MasterReceiveAck((XMC_USIC_CH_t*)obj_s->i2c);

	while(XMC_USIC_CH_RXFIFO_IsEmpty((XMC_USIC_CH_t*)obj_s->i2c));

	return XMC_I2C_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->i2c);
}

int i2c_byte_write(i2c_t *obj, int data)
{
	int timeout;
	struct i2c_s *obj_s = I2C_S(obj);

	while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));

	if (obj_s->start_condition_byte)
	{
		XMC_I2C_CH_MasterStart((XMC_USIC_CH_t*)obj_s->i2c, data, XMC_I2C_CH_CMD_WRITE);
		obj_s->start_condition_byte = 0;
	}
	else
		XMC_I2C_CH_MasterTransmit((XMC_USIC_CH_t*)obj_s->i2c, data);

	timeout = FLAG_TIMEOUT;

	while(timeout--)
	{
		uint32_t flag = XMC_I2C_CH_GetStatusFlag((XMC_USIC_CH_t*)obj_s->i2c);

		if (flag & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED)
		{
			XMC_I2C_CH_ClearStatusFlag((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
			return 1;
		}
		else if (flag & XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED)
		{
			XMC_I2C_CH_ClearStatusFlag((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED);
			return 0;
		}
	}

	return 2;
}

#if DEVICE_I2CSLAVE

// See I2CSlave.h
#define NoData         0 // the slave has not been addressed
#define ReadAddressed  1 // the master has requested a read from this slave (slave = transmitter)
#define WriteGeneral   2 // the master is writing to all slave
#define WriteAddressed 3 // the master is writing to this slave (slave = receiver)

void i2c_slave_mode(i2c_t *obj, int enable_slave) {

    struct i2c_s *obj_s = I2C_S(obj);

    if (enable_slave) {
        obj_s->slave = 1;
    } else {
        obj_s->slave = 0;
    }
}

void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask)
{
	struct i2c_s *obj_s = I2C_S(obj);

	obj_s->address = address;

	while (XMC_I2C_CH_Stop((XMC_USIC_CH_t*)obj_s->i2c) == XMC_I2C_CH_STATUS_BUSY);

	XMC_I2C_CH_CONFIG_t config =
	{
		.baudrate = 100000,
		.address = address,
	};
	XMC_I2C_CH_Init((XMC_USIC_CH_t*)obj_s->i2c, &config);
	XMC_I2C_CH_Start((XMC_USIC_CH_t*)obj_s->i2c);
}

int i2c_slave_receive(i2c_t *obj)
{
	struct i2c_s *obj_s = I2C_S(obj);
	int retValue = NoData;

	if (XMC_I2C_CH_GetStatusFlag((XMC_USIC_CH_t*)obj_s->i2c) & XMC_I2C_CH_STATUS_FLAG_SLAVE_READ_REQUESTED)
	{
		retValue = ReadAddressed;
		XMC_I2C_CH_ClearStatusFlag((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_STATUS_FLAG_SLAVE_READ_REQUESTED);
	}

	return retValue;
}

int i2c_slave_write(i2c_t *obj, const char *data, int length)
{
	struct i2c_s* obj_s = I2C_S(obj);

	for (int i=0; i<length; i++)
	{
		while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
		XMC_I2C_CH_SlaveTransmit((XMC_USIC_CH_t*)obj_s->i2c, data[i]);
	}

	return 1;
}

#endif

#if DEVICE_I2C_ASYNCH

static IRQn_Type i2c_get_transfer_irq_n(i2c_t *obj)
{
    struct i2c_s *obj_s = I2C_S(obj);
    IRQn_Type irq_n;

    switch (obj_s->index) {
        case 0:
            irq_n = USIC0_0_IRQn;
            break;
        case 1:
            irq_n = USIC0_1_IRQn;
            break;
        case 2:
            irq_n = USIC1_0_IRQn;
            break;
        case 3:
            irq_n = USIC1_1_IRQn;
            break;
        case 4:
            irq_n = USIC2_0_IRQn;
            break;
        case 5:
            irq_n = USIC2_1_IRQn;
            break;
        default:
            irq_n = (IRQn_Type)0;
    }

    return irq_n;
}

void i2c_transfer_asynch(i2c_t *obj, const void *tx, size_t tx_length, void *rx, size_t rx_length, uint32_t address, uint32_t stop, uint32_t handler, uint32_t event, DMAUsage hint)
{
    // TODO: DMA usage is currently ignored
    (void) hint;

    MBED_ASSERT(tx != (void*)0);
    MBED_ASSERT(rx != (void*)0);

    struct i2c_s *obj_s = I2C_S(obj);

    if (tx_length == 0)
    	return;
    if (rx_length == 0)
    	return;

    obj->tx_buff.buffer = (void*)tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos = 0;
    obj->tx_buff.width = 8;

    obj->rx_buff.buffer = (void*)rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos = 0;
    obj->rx_buff.width = 8;

    obj_s->available_events = event;
    obj_s->address = address;
    obj_s->stop = stop;

    IRQn_Type irq_n = i2c_get_transfer_irq_n(obj);
    NVIC_ClearPendingIRQ(irq_n);
    NVIC_DisableIRQ(irq_n);
    NVIC_SetVector(irq_n, (uint32_t)handler);
    NVIC_EnableIRQ(irq_n);

    obj_s->transfer_busy = 1;

    XMC_USIC_CH_RXFIFO_Flush((XMC_USIC_CH_t*)obj_s->i2c);
    XMC_USIC_CH_TXFIFO_Flush((XMC_USIC_CH_t*)obj_s->i2c);

    XMC_I2C_CH_EnableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_EVENT_ACK);
    XMC_I2C_CH_EnableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_EVENT_NACK);
    XMC_I2C_CH_EnableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_EVENT_ERROR);

    if (obj_s->repeated_start)
    	XMC_I2C_CH_MasterRepeatedStart((XMC_USIC_CH_t*)obj_s->i2c, obj_s->address, XMC_I2C_CH_CMD_WRITE);
    else
    	XMC_I2C_CH_MasterStart((XMC_USIC_CH_t*)obj_s->i2c, obj_s->address, XMC_I2C_CH_CMD_WRITE);

    obj_s->async_state = ASYNCH_Start;
}

uint32_t i2c_irq_handler_asynch(i2c_t *obj)
{
	uint32_t event = 0;
	XMC_I2C_CH_STATUS_FLAG_t flag;

	struct i2c_s *obj_s = I2C_S(obj);

	flag = XMC_I2C_CH_GetStatusFlag((XMC_USIC_CH_t*)obj_s->i2c);

	if (flag & XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED)
	{
		event = I2C_EVENT_ERROR_NO_SLAVE;
		XMC_I2C_CH_ClearStatusFlag((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED);
		i2c_abort_asynch(obj);
	}
	else if (flag & XMC_I2C_CH_STATUS_FLAG_ERROR)
	{
		event = I2C_EVENT_ERROR;
		XMC_I2C_CH_ClearStatusFlag((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_STATUS_FLAG_ERROR);
		i2c_abort_asynch(obj);
	}
	else
	{
		switch (obj_s->async_state)
		{
			case ASYNCH_Start:
				if (flag & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED)
				{
					XMC_I2C_CH_DisableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_EVENT_ACK);
					XMC_USIC_CH_TXFIFO_EnableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);

					while (obj->tx_buff.pos < obj->tx_buff.length)
					{
						while (XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
						XMC_I2C_CH_MasterTransmit((XMC_USIC_CH_t*)obj_s->i2c, ((uint8_t*)obj->tx_buff.buffer)[obj->tx_buff.pos++]);
					}

					obj_s->async_state = ASYNCH_Transmit;

					XMC_I2C_CH_ClearStatusFlag((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
				}
				break;
			case ASYNCH_Transmit:
				if (XMC_USIC_CH_TXFIFO_GetEvent((XMC_USIC_CH_t*)obj_s->i2c) & XMC_USIC_CH_TXFIFO_EVENT_STANDARD)
				{
					XMC_USIC_CH_TXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
					XMC_USIC_CH_RXFIFO_EnableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE | XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD);

					XMC_I2C_CH_MasterRepeatedStart((XMC_USIC_CH_t*)obj_s->i2c, obj_s->address, XMC_I2C_CH_CMD_READ);

					if (obj->rx_buff.length > HALF_FIFO_SIZE)
					{
						XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_FIFO_SIZE_16WORDS, HALF_FIFO_SIZE-1);
						for (uint8_t i=0; i<HALF_FIFO_SIZE; i++)
						{
							while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
							XMC_I2C_CH_MasterReceiveAck((XMC_USIC_CH_t*)obj_s->i2c);
						}
					}
					else
					{
						XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_FIFO_SIZE_16WORDS, obj->rx_buff.length-1);
						for (uint8_t i=0; i<obj->rx_buff.length-1; i++)
							XMC_I2C_CH_MasterReceiveAck((XMC_USIC_CH_t*)obj_s->i2c);

						XMC_I2C_CH_MasterReceiveNack((XMC_USIC_CH_t*)obj_s->i2c);
					}

					obj_s->async_state = ASYNCH_Receive;

					XMC_USIC_CH_TXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_TXFIFO_EVENT_STANDARD);
				}
				break;
			case ASYNCH_Receive:
				if ((XMC_USIC_CH_RXFIFO_GetEvent((XMC_USIC_CH_t*)obj_s->i2c) & XMC_USIC_CH_RXFIFO_EVENT_ALTERNATE) |
					(XMC_USIC_CH_RXFIFO_GetEvent((XMC_USIC_CH_t*)obj_s->i2c) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD))
				{
					uint8_t level = XMC_USIC_CH_RXFIFO_GetLevel((XMC_USIC_CH_t*)obj_s->i2c);

					for (uint8_t i=0; i<level; i++)
						((uint8_t*)obj->rx_buff.buffer)[obj->rx_buff.pos++] = XMC_I2C_CH_GetReceivedData((XMC_USIC_CH_t*)obj_s->i2c);

					if (obj->rx_buff.pos >= obj->rx_buff.length)
					{
						XMC_USIC_CH_RXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE | XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD);

						if (obj_s->stop)
						{
							XMC_I2C_CH_MasterStop((XMC_USIC_CH_t*)obj_s->i2c);
							obj_s->repeated_start = 0;
						}
						else
							obj_s->repeated_start = 1;

						obj_s->async_state = ASYNCH_Stop;
						obj_s->transfer_busy = 0;

						event = I2C_EVENT_TRANSFER_COMPLETE;
					}
					else
					{
						uint8_t size = obj->rx_buff.length - obj->rx_buff.pos;

						if (size > HALF_FIFO_SIZE)
						{
							for (uint8_t i=0; i<HALF_FIFO_SIZE; i++)
							{
								while(XMC_USIC_CH_TXFIFO_IsFull((XMC_USIC_CH_t*)obj_s->i2c));
								XMC_I2C_CH_MasterReceiveAck((XMC_USIC_CH_t*)obj_s->i2c);
							}
						}
						else
						{
							XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_FIFO_SIZE_16WORDS, size-1);
							for (uint8_t i=0; i<size-1; i++)
								XMC_I2C_CH_MasterReceiveAck((XMC_USIC_CH_t*)obj_s->i2c);

							XMC_I2C_CH_MasterReceiveNack((XMC_USIC_CH_t*)obj_s->i2c);
						}
					}

					XMC_USIC_CH_RXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_RXFIFO_EVENT_ALTERNATE | XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
				}
				break;
			default:
				break;
		}
	}

	return (event & obj_s->available_events);
}

uint8_t i2c_active(i2c_t *obj)
{
	struct i2c_s *obj_s = I2C_S(obj);

	return obj_s->transfer_busy;
}

void i2c_abort_asynch(i2c_t *obj)
{
	struct i2c_s *obj_s = I2C_S(obj);

	XMC_USIC_CH_TXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
	XMC_USIC_CH_TXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_TXFIFO_EVENT_STANDARD);

	XMC_USIC_CH_RXFIFO_DisableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE | XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD);
	XMC_USIC_CH_RXFIFO_ClearEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_USIC_CH_RXFIFO_EVENT_ALTERNATE | XMC_USIC_CH_RXFIFO_EVENT_STANDARD);

	XMC_I2C_CH_DisableEvent((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_EVENT_ACK);
	XMC_I2C_CH_ClearStatusFlag((XMC_USIC_CH_t*)obj_s->i2c, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

	XMC_USIC_CH_RXFIFO_Flush((XMC_USIC_CH_t*)obj_s->i2c);
	XMC_USIC_CH_TXFIFO_Flush((XMC_USIC_CH_t*)obj_s->i2c);

	obj_s->transfer_busy = 0;
}

#endif //DEVICE_I2C_ASYNCH

#endif //DEVICE_I2C
