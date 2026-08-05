/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the BCAN PROTOL codec.
 *
 * Tests the pure encode/decode functions without any UART hardware.
 */

#include <asr/bcan_protol_internal.h>

#include <string.h>

#include <zephyr/ztest.h>

/* -------------------------------------------------------------------------
 * Helper: build a CAN frame
 * ------------------------------------------------------------------------- */

static void make_frame(struct asr_can_frame *frame,
		       uint32_t can_id, uint8_t dlc,
		       uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
		       uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
	memset(frame, 0, sizeof(*frame));
	frame->can_id = can_id;
	frame->dlc = dlc;
	uint8_t data[] = {d0, d1, d2, d3, d4, d5, d6, d7};

	memcpy(frame->data, data, dlc < 8 ? dlc : 8);
}

/* -------------------------------------------------------------------------
 * Encode tests
 * ------------------------------------------------------------------------- */

ZTEST(bcan_protol_codec, test_encode_dlc8)
{
	struct asr_can_frame frame;
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];

	make_frame(&frame, 0x201, 8, 0x11, 0x22, 0x33, 0x44,
		   0x55, 0x66, 0x77, 0x88);

	int ret = asr_bcan_protol_encode(&frame, raw);

	zassert_equal(ret, 0, "encode should succeed");
	zassert_equal(raw[0], 0x08, "frame_info should be 0x08 (DLC=8)");
	zassert_equal(raw[1], 0x00, "CAN ID byte 1");
	zassert_equal(raw[2], 0x00, "CAN ID byte 2");
	zassert_equal(raw[3], 0x02, "CAN ID byte 3");
	zassert_equal(raw[4], 0x01, "CAN ID byte 4");
	zassert_equal(raw[5], 0x11, "DATA[0]");
	zassert_equal(raw[6], 0x22, "DATA[1]");
	zassert_equal(raw[7], 0x33, "DATA[2]");
	zassert_equal(raw[8], 0x44, "DATA[3]");
	zassert_equal(raw[9], 0x55, "DATA[4]");
	zassert_equal(raw[10], 0x66, "DATA[5]");
	zassert_equal(raw[11], 0x77, "DATA[6]");
	zassert_equal(raw[12], 0x88, "DATA[7]");
}

ZTEST(bcan_protol_codec, test_encode_dlc4_pads_zeros)
{
	struct asr_can_frame frame;
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];

	make_frame(&frame, 0x123, 4, 0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0);

	int ret = asr_bcan_protol_encode(&frame, raw);

	zassert_equal(ret, 0, "encode should succeed");
	zassert_equal(raw[0], 0x04, "frame_info should be 0x04 (DLC=4)");
	zassert_equal(raw[5], 0xAA, "DATA[0]");
	zassert_equal(raw[6], 0xBB, "DATA[1]");
	zassert_equal(raw[7], 0xCC, "DATA[2]");
	zassert_equal(raw[8], 0xDD, "DATA[3]");
	/* Bytes 9-12 should be zero-padded. */
	zassert_equal(raw[9], 0x00, "DATA[4] should be zero");
	zassert_equal(raw[10], 0x00, "DATA[5] should be zero");
	zassert_equal(raw[11], 0x00, "DATA[6] should be zero");
	zassert_equal(raw[12], 0x00, "DATA[7] should be zero");
}

ZTEST(bcan_protol_codec, test_encode_dlc0_all_zero)
{
	struct asr_can_frame frame;
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];

	make_frame(&frame, 0x7FF, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	int ret = asr_bcan_protol_encode(&frame, raw);

	zassert_equal(ret, 0, "encode should succeed");
	zassert_equal(raw[0], 0x00, "frame_info should be 0x00 (DLC=0)");
	/* All data bytes should be zero. */
	for (int i = 5; i < 13; i++) {
		zassert_equal(raw[i], 0x00, "DATA[%d] should be zero", i - 5);
	}
}

ZTEST(bcan_protol_codec, test_encode_dlc_overflow)
{
	struct asr_can_frame frame;
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];

	make_frame(&frame, 0x201, 9, 0, 0, 0, 0, 0, 0, 0, 0);

	int ret = asr_bcan_protol_encode(&frame, raw);

	zassert_equal(ret, -EMSGSIZE, "DLC>8 should return -EMSGSIZE");
}

ZTEST(bcan_protol_codec, test_encode_can_id_overflow)
{
	struct asr_can_frame frame;
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];

	make_frame(&frame, 0x800, 8, 0, 0, 0, 0, 0, 0, 0, 0);

	int ret = asr_bcan_protol_encode(&frame, raw);

	zassert_equal(ret, -ERANGE, "CAN ID>0x7FF should return -ERANGE");
}

ZTEST(bcan_protol_codec, test_encode_can_id_boundaries)
{
	struct asr_can_frame frame;
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];

	/* CAN ID = 0x000 (minimum) */
	make_frame(&frame, 0x000, 8, 0, 0, 0, 0, 0, 0, 0, 0);
	zassert_equal(asr_bcan_protol_encode(&frame, raw), 0, "CAN ID=0 should succeed");
	zassert_equal(raw[1], 0x00, "CAN ID=0 byte 1");
	zassert_equal(raw[4], 0x00, "CAN ID=0 byte 4");

	/* CAN ID = 0x7FF (maximum) */
	make_frame(&frame, 0x7FF, 8, 0, 0, 0, 0, 0, 0, 0, 0);
	zassert_equal(asr_bcan_protol_encode(&frame, raw), 0, "CAN ID=0x7FF should succeed");
	zassert_equal(raw[3], 0x07, "CAN ID=0x7FF byte 3");
	zassert_equal(raw[4], 0xFF, "CAN ID=0x7FF byte 4");
}

ZTEST(bcan_protol_codec, test_encode_null_parameters)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];
	struct asr_can_frame frame;

	memset(&frame, 0, sizeof(frame));

	zassert_equal(asr_bcan_protol_encode(NULL, raw), -EINVAL,
		      "NULL frame should return -EINVAL");
	zassert_equal(asr_bcan_protol_encode(&frame, NULL), -EINVAL,
		      "NULL raw should return -EINVAL");
}

/* -------------------------------------------------------------------------
 * Decode tests
 * ------------------------------------------------------------------------- */

ZTEST(bcan_protol_codec, test_decode_standard_frame)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {
		0x08,                /* DLC=8, no EXT, no RTR */
		0x00, 0x00, 0x02, 0x01,  /* CAN ID = 0x201 */
		0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, /* DATA */
	};
	struct asr_can_frame frame;

	int ret = asr_bcan_protol_decode(raw, &frame);

	zassert_equal(ret, 0, "decode should succeed");
	zassert_equal(frame.can_id, 0x201, "CAN ID");
	zassert_equal(frame.dlc, 8, "DLC");
	zassert_equal(frame.data[0], 0x11, "DATA[0]");
	zassert_equal(frame.data[7], 0x88, "DATA[7]");
}

ZTEST(bcan_protol_codec, test_decode_ext_frame_rejected)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {0};
	struct asr_can_frame frame;

	raw[0] = 0x88; /* EXT=1, DLC=8 */

	int ret = asr_bcan_protol_decode(raw, &frame);

	zassert_equal(ret, -ENOTSUP, "EXT frame should return -ENOTSUP");
}

ZTEST(bcan_protol_codec, test_decode_rtr_frame_rejected)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {0};
	struct asr_can_frame frame;

	raw[0] = 0x48; /* RTR=1, DLC=8 */

	int ret = asr_bcan_protol_decode(raw, &frame);

	zassert_equal(ret, -ENOTSUP, "RTR frame should return -ENOTSUP");
}

ZTEST(bcan_protol_codec, test_decode_dlc_overflow)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {0};
	struct asr_can_frame frame;

	raw[0] = 0x09; /* DLC=9 (too large) */

	int ret = asr_bcan_protol_decode(raw, &frame);

	zassert_equal(ret, -EMSGSIZE, "DLC>8 should return -EMSGSIZE");
}

ZTEST(bcan_protol_codec, test_decode_can_id_overflow)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {
		0x08,                /* DLC=8, no EXT, no RTR */
		0x00, 0x00, 0x08, 0x00,  /* CAN ID = 0x800 (> 0x7FF) */
	};
	struct asr_can_frame frame;

	int ret = asr_bcan_protol_decode(raw, &frame);

	zassert_equal(ret, -ERANGE, "CAN ID>0x7FF should return -ERANGE");
}

ZTEST(bcan_protol_codec, test_decode_can_id_boundaries)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {0};
	struct asr_can_frame frame;

	/* CAN ID = 0x000 (minimum) */
	raw[0] = 0x08;
	raw[1] = 0x00; raw[2] = 0x00; raw[3] = 0x00; raw[4] = 0x00;
	zassert_equal(asr_bcan_protol_decode(raw, &frame), 0, "CAN ID=0 should succeed");
	zassert_equal(frame.can_id, 0, "CAN ID should be 0");

	/* CAN ID = 0x7FF (maximum) */
	raw[3] = 0x07; raw[4] = 0xFF;
	zassert_equal(asr_bcan_protol_decode(raw, &frame), 0, "CAN ID=0x7FF should succeed");
	zassert_equal(frame.can_id, 0x7FF, "CAN ID should be 0x7FF");
}

ZTEST(bcan_protol_codec, test_decode_short_dlc)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {
		0x04,                /* DLC=4 */
		0x00, 0x00, 0x01, 0x23,  /* CAN ID = 0x123 */
		0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0,
	};
	struct asr_can_frame frame;

	int ret = asr_bcan_protol_decode(raw, &frame);

	zassert_equal(ret, 0, "decode DLC=4 should succeed");
	zassert_equal(frame.dlc, 4, "DLC should be 4");
	zassert_equal(frame.data[0], 0xAA, "DATA[0]");
	zassert_equal(frame.data[3], 0xDD, "DATA[3]");
	/* Bytes beyond DLC should be zero. */
	zassert_equal(frame.data[4], 0x00, "DATA[4] should be zero");
}

ZTEST(bcan_protol_codec, test_decode_null_parameters)
{
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN] = {0};
	struct asr_can_frame frame;

	zassert_equal(asr_bcan_protol_decode(NULL, &frame), -EINVAL,
		      "NULL raw should return -EINVAL");
	zassert_equal(asr_bcan_protol_decode(raw, NULL), -EINVAL,
		      "NULL frame should return -EINVAL");
}

/* -------------------------------------------------------------------------
 * Round-trip test
 * ------------------------------------------------------------------------- */

ZTEST(bcan_protol_codec, test_encode_decode_roundtrip)
{
	struct asr_can_frame original;
	struct asr_can_frame decoded;
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN];

	make_frame(&original, 0x555, 8, 0x12, 0x34, 0x56, 0x78,
		   0x9A, 0xBC, 0xDE, 0xF0);

	zassert_equal(asr_bcan_protol_encode(&original, raw), 0, "encode");
	zassert_equal(asr_bcan_protol_decode(raw, &decoded), 0, "decode");

	zassert_equal(decoded.can_id, original.can_id, "CAN ID round-trip");
	zassert_equal(decoded.dlc, original.dlc, "DLC round-trip");
	zassert_mem_equal(decoded.data, original.data, 8, "DATA round-trip");
}

/* -------------------------------------------------------------------------
 * Test suite
 * ------------------------------------------------------------------------- */

ZTEST_SUITE(bcan_protol_codec, NULL, NULL, NULL, NULL, NULL);