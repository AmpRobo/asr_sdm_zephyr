/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART-AA55 transport layer for the ASR SDM protocol.
 *
 * Interrupt-driven RX reassembles framed messages through a byte-level state
 * machine and enqueues complete 8-byte payloads.  TX uses blocking poll-out
 * with a mutex for thread safety.
 *
 * Wire frame:
 *   [0xAA] [0x55] [LEN] [DATA_0 .. DATA_N-1] [CHK]
 *   CHK = LEN ^ DATA_0 ^ ... ^ DATA_N-1
 */

#include <asr/transport_uart_aa55.h>
#include <asr/protocol_core.h>

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_comm, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * UART device
 * ------------------------------------------------------------------------- */

#define COMM_UART_NODE DT_NODELABEL(uart0)

BUILD_ASSERT(DT_NODE_EXISTS(COMM_UART_NODE),
	     "uart0 node must exist in devicetree");

static const struct device *const uart_dev =
	DEVICE_DT_GET(COMM_UART_NODE);

/* -------------------------------------------------------------------------
 * Frame constants
 * ------------------------------------------------------------------------- */

#define FRAME_SYNC_HI         0xAAU
#define FRAME_SYNC_LO         0x55U
#define FRAME_MAX_PAYLOAD     ASR_COMM_MSG_SIZE

/* -------------------------------------------------------------------------
 * RX state machine
 * ------------------------------------------------------------------------- */

enum rx_state {
	RX_SYNC1,
	RX_SYNC2,
	RX_LENGTH,
	RX_DATA,
	RX_CHECKSUM,
};

K_MSGQ_DEFINE(rx_msgq, ASR_COMM_MSG_SIZE,
	      CONFIG_ASR_COMM_RX_QUEUE_DEPTH, 4);

static struct {
	enum rx_state state;
	uint8_t buf[FRAME_MAX_PAYLOAD];
	uint8_t expected_len;
	uint8_t idx;
	uint8_t checksum;
} rx_ctx;

static void rx_process_byte(uint8_t byte)
{
	switch (rx_ctx.state) {
	case RX_SYNC1:
		if (byte == FRAME_SYNC_HI) {
			rx_ctx.state = RX_SYNC2;
		}
		break;

	case RX_SYNC2:
		rx_ctx.state = (byte == FRAME_SYNC_LO) ? RX_LENGTH : RX_SYNC1;
		break;

	case RX_LENGTH:
		if (byte == 0U || byte > FRAME_MAX_PAYLOAD) {
			rx_ctx.state = RX_SYNC1;
			break;
		}
		rx_ctx.expected_len = byte;
		rx_ctx.idx = 0U;
		rx_ctx.checksum = byte;
		rx_ctx.state = RX_DATA;
		break;

	case RX_DATA:
		rx_ctx.buf[rx_ctx.idx++] = byte;
		rx_ctx.checksum ^= byte;
		if (rx_ctx.idx >= rx_ctx.expected_len) {
			rx_ctx.state = RX_CHECKSUM;
		}
		break;

	case RX_CHECKSUM:
		if (byte == rx_ctx.checksum) {
			if (rx_ctx.expected_len < ASR_COMM_MSG_SIZE) {
				memset(&rx_ctx.buf[rx_ctx.expected_len], 0,
				       ASR_COMM_MSG_SIZE - rx_ctx.expected_len);
			}
			k_msgq_put(&rx_msgq, rx_ctx.buf, K_NO_WAIT);
		} else {
			LOG_WRN("frame checksum mismatch");
		}
		rx_ctx.state = RX_SYNC1;
		break;
	}
}

static void uart_isr_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		uint8_t byte;

		while (uart_fifo_read(dev, &byte, 1) == 1) {
			rx_process_byte(byte);
		}
	}
}

/* -------------------------------------------------------------------------
 * TX mutex (poll-out is not thread-safe without it)
 * ------------------------------------------------------------------------- */

static K_MUTEX_DEFINE(tx_mutex);

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

int asr_uart_aa55_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART0 device not ready");
		return -ENODEV;
	}

	uart_irq_callback_set(uart_dev, uart_isr_cb);
	uart_irq_rx_enable(uart_dev);

	LOG_INF("UART0 AA55 transport initialised");
	return 0;
}

int asr_uart_aa55_receive(uint8_t message[ASR_COMM_MSG_SIZE],
			  k_timeout_t timeout)
{
	return k_msgq_get(&rx_msgq, message, timeout);
}

int asr_uart_aa55_send(const uint8_t message[ASR_COMM_MSG_SIZE])
{
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	k_mutex_lock(&tx_mutex, K_FOREVER);

	uint8_t checksum = ASR_COMM_MSG_SIZE;

	uart_poll_out(uart_dev, FRAME_SYNC_HI);
	uart_poll_out(uart_dev, FRAME_SYNC_LO);
	uart_poll_out(uart_dev, ASR_COMM_MSG_SIZE);

	for (int i = 0; i < ASR_COMM_MSG_SIZE; i++) {
		uart_poll_out(uart_dev, message[i]);
		checksum ^= message[i];
	}

	uart_poll_out(uart_dev, checksum);

	k_mutex_unlock(&tx_mutex);
	return 0;
}

/* -------------------------------------------------------------------------
 * Legacy API wrappers (for backward compatibility)
 * ------------------------------------------------------------------------- */

int asr_comm_send(const uint8_t data[ASR_COMM_MSG_SIZE])
{
	return asr_uart_aa55_send(data);
}

void asr_comm_register_callbacks(const struct asr_comm_callbacks *cb)
{
	asr_protocol_core_register_callbacks(cb);
}

const asr_unit_status_t *asr_comm_get_status(void)
{
	return asr_protocol_core_get_status();
}

bool protocol_init(void)
{
	return asr_protocol_core_init() == 0;
}