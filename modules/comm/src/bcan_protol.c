/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCAN-S01 PROTOL transport layer for the ASR SDM protocol.
 *
 * Pure transport layer: encodes/decodes 13-byte PROTOL frames on UART0
 * into generic struct asr_can_frame.  No business logic, Dynamixel calls,
 * or protocol command dispatch.
 *
 * Uses poll-based UART I/O with inter-byte timeout.  A future upgrade may
 * switch to ISR or async UART (at which point a message queue between the
 * ISR and the processing thread would be added).
 */

#include <asr/bcan_protol.h>
#include <asr/bcan_protol_internal.h>

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_bcan_protol, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * UART device
 * ------------------------------------------------------------------------- */

#define BCAN_UART_NODE DT_NODELABEL(uart0)

BUILD_ASSERT(DT_NODE_EXISTS(BCAN_UART_NODE),
	     "uart0 node must exist for BCAN PROTOL transport");

static const struct device *const uart_dev =
	DEVICE_DT_GET(BCAN_UART_NODE);

/* -------------------------------------------------------------------------
 * Transport statistics
 * ------------------------------------------------------------------------- */

static struct asr_bcan_protol_stats stats;

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static int uart_read_byte(uint8_t *byte)
{
	int64_t deadline = k_uptime_get() + CONFIG_ASR_BCAN_INTERBYTE_TIMEOUT_MS;

	while (k_uptime_get() < deadline) {
		if (uart_poll_in(uart_dev, byte) == 0) {
			return 0;
		}
		k_usleep(100U);
	}

	++stats.uart_errors;
	return -ETIMEDOUT;
}

/*
 * Read a complete 13-byte PROTOL record from UART.
 *
 * Synchronisation: waits for a byte where frame_info == 0x08
 * (standard data frame, DLC=8, no EXT or RTR), then reads the
 * remaining 12 bytes.  If the decoded frame is invalid, retries.
 *
 * This is a heuristic, not a guaranteed sync -- PROTOL has no
 * dedicated frame header.  False positives are possible when
 * payload bytes equal 0x08, but the subsequent decode_frame
 * validation (CAN ID range, DLC, EXT/RTR flags) catches most
 * misalignments.
 */
static int read_raw_record(uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN])
{
	uint8_t byte;
	int ret;

	while (true) {
		/* Wait for a byte that looks like a frame_info header. */
		ret = uart_read_byte(&byte);
		if (ret < 0) {
			return ret;
		}

		/* Accept only standard data frames with DLC = 8. */
		if (byte != 0x08U) {
			continue;
		}

		raw[0] = byte;

		/* Read the remaining 12 bytes. */
		for (size_t i = 1U; i < ASR_BCAN_PROTOL_FRAME_LEN; ++i) {
			ret = uart_read_byte(&raw[i]);
			if (ret < 0) {
				return ret;
			}
		}

		/* Validate the complete record. */
		struct asr_can_frame tmp;

		ret = asr_bcan_protol_decode(raw, &tmp);
		if (ret < 0) {
			++stats.decode_errors;
			continue;
		}

		++stats.frames_received;
		return 0;
	}
}

static void uart_write_all(const uint8_t *data, size_t len)
{
	for (size_t i = 0U; i < len; ++i) {
		uart_poll_out(uart_dev, data[i]);
	}
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

int asr_bcan_protol_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("BCAN UART0 device not ready");
		return -ENODEV;
	}

	LOG_INF("BCAN PROTOL transport initialised on UART0");
	return 0;
}

int asr_bcan_protol_receive(struct asr_can_frame *frame,
			    k_timeout_t timeout)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];
	int ret;

	ret = read_raw_record(raw);
	if (ret < 0) {
		if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			return -EAGAIN;
		}
		return ret;
	}

	return asr_bcan_protol_decode(raw, frame);
}

int asr_bcan_protol_send(const struct asr_can_frame *frame)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];
	int ret;

	ret = asr_bcan_protol_encode(frame, raw);
	if (ret < 0) {
		return ret;
	}

	uart_write_all(raw, sizeof(raw));
	++stats.frames_sent;
	return 0;
}

const struct asr_bcan_protol_stats *asr_bcan_protol_get_stats(void)
{
	return &stats;
}