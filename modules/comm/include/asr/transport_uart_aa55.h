/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART-AA55 transport layer for the ASR SDM protocol.
 *
 * Interrupt-driven RX reassembles [0xAA 0x55 LEN DATA... CHK] frames through
 * a byte-level state machine and enqueues complete 8-byte payloads.  TX uses
 * blocking poll-out protected by a mutex.
 */

#ifndef ASR_TRANSPORT_UART_AA55_H_
#define ASR_TRANSPORT_UART_AA55_H_

#include <stdint.h>

#include <zephyr/kernel.h>
#include <asr/comm_thread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise UART0 and enable interrupt-driven RX.
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_uart_aa55_init(void);

/**
 * Block until a complete 8-byte protocol message is received.
 *
 * @param message  Output buffer of exactly ASR_COMM_MSG_SIZE bytes.
 * @param timeout  Maximum time to wait (K_FOREVER to block indefinitely).
 * @return 0 on success, -EAGAIN on timeout, negative errno on other errors.
 */
int asr_uart_aa55_receive(uint8_t message[ASR_COMM_MSG_SIZE],
			  k_timeout_t timeout);

/**
 * Transmit an 8-byte protocol message, framed as:
 *   [0xAA] [0x55] [LEN=8] [DATA...] [CHK = LEN ^ DATA_0 ^ ... ^ DATA_7]
 *
 * This function is thread-safe (protected by an internal mutex).
 *
 * @param message  Pointer to exactly ASR_COMM_MSG_SIZE bytes to send.
 * @return 0 on success, negative errno on failure.
 */
int asr_uart_aa55_send(const uint8_t message[ASR_COMM_MSG_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* ASR_TRANSPORT_UART_AA55_H_ */