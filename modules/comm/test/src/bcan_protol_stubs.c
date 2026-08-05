/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCAN PROTOL stubs for unit testing.
 *
 * Provides minimal implementations of the BCAN PROTOL transport functions
 * so that can_protocol_transport.c can be linked without UART hardware.
 */

#include <asr/bcan_protol.h>

#include <errno.h>

int asr_bcan_protol_init(void)
{
	return 0;
}

int asr_bcan_protol_receive(struct asr_can_frame *frame, k_timeout_t timeout)
{
	ARG_UNUSED(frame);
	ARG_UNUSED(timeout);
	return -ENOTSUP;
}

int asr_bcan_protol_send(const struct asr_can_frame *frame)
{
	ARG_UNUSED(frame);
	return 0;
}

const struct asr_bcan_protol_stats *asr_bcan_protol_get_stats(void)
{
	return NULL;
}