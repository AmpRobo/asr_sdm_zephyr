/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the CAN protocol transport business mapping function.
 *
 * Tests asr_can_protocol_handle_frame() with fake callbacks, verifying
 * CAN ID filtering, DLC validation, and protocol_core integration.
 * No UART or BCAN hardware required.
 */

#include <asr/can_protocol_transport.h>
#include <asr/protocol_core.h>

#include <string.h>

#include <zephyr/ztest.h>

/* -------------------------------------------------------------------------
 * Fake callback state
 * ------------------------------------------------------------------------- */

static struct {
	bool imu_read_called;
	uint8_t imu_read_response[ASR_COMM_MSG_SIZE];

	bool position_read_called;
	int32_t position_read_value;
} fake;

static void reset_fake(void)
{
	memset(&fake, 0, sizeof(fake));
}

static int fake_on_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE])
{
	fake.imu_read_called = true;
	memcpy(buf, fake.imu_read_response, ASR_COMM_MSG_SIZE);
	return 0;
}

static int fake_on_dynamixel_position_read(uint8_t id, int32_t *position)
{
	ARG_UNUSED(id);
	fake.position_read_called = true;
	*position = fake.position_read_value;
	return 0;
}

static const struct asr_comm_callbacks test_callbacks = {
	.on_imu_read = fake_on_imu_read,
	.on_dynamixel_position_read = fake_on_dynamixel_position_read,
};

/* Request / response CAN IDs matching screw_unit configuration. */
#define TEST_REQ_ID 0x123
#define TEST_RESP_ID 0x124

/* -------------------------------------------------------------------------
 * Setup
 * ------------------------------------------------------------------------- */

static void *setup(void)
{
	asr_protocol_core_init();
	asr_protocol_core_register_callbacks(&test_callbacks);
	asr_can_protocol_test_set_ids(TEST_REQ_ID, TEST_RESP_ID);
	return NULL;
}

static void before_test(void *fixture)
{
	ARG_UNUSED(fixture);
	reset_fake();
}

/* -------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static void make_can_frame(struct asr_can_frame *frame,
			   uint32_t can_id, uint8_t dlc,
			   uint8_t cmd, uint8_t param)
{
	memset(frame, 0, sizeof(*frame));
	frame->can_id = can_id;
	frame->dlc = dlc;
	frame->data[2] = cmd;
	frame->data[3] = param;
}

/* -------------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------------- */

ZTEST(can_protocol_transport, test_correct_request_id)
{
	struct asr_can_frame request;
	struct asr_can_frame response;
	bool has_response;

	/* IMU READ: should produce a response. */
	fake.imu_read_response[0] = 0xAA;
	make_can_frame(&request, TEST_REQ_ID, 8,
		       ASR_COMM_CMD_READ, ASR_COMM_PARAM_IMU);

	int ret = asr_can_protocol_handle_frame(&request, &response,
						&has_response);

	zassert_equal(ret, 0, "handle_frame should succeed");
	zassert_true(has_response, "IMU READ should have response");
	zassert_true(fake.imu_read_called, "on_imu_read should be called");
	zassert_equal(response.can_id, TEST_RESP_ID,
		      "response CAN ID should match");
	zassert_equal(response.dlc, 8, "response DLC should be 8");
	zassert_equal(response.data[0], 0xAA, "response data should match");
}

ZTEST(can_protocol_transport, test_wrong_request_id)
{
	struct asr_can_frame request;
	struct asr_can_frame response;
	bool has_response;

	/* Send a frame with a different CAN ID. */
	make_can_frame(&request, 0x456, 8,
		       ASR_COMM_CMD_READ, ASR_COMM_PARAM_IMU);

	int ret = asr_can_protocol_handle_frame(&request, &response,
						&has_response);

	zassert_equal(ret, 0, "wrong CAN ID should return 0 (silently ignored)");
	zassert_false(has_response, "no response expected");
	zassert_false(fake.imu_read_called, "callback should not be called");
}

ZTEST(can_protocol_transport, test_joint1_read_position_response)
{
	struct asr_can_frame request;
	struct asr_can_frame response;
	bool has_response;

	fake.position_read_value = 0x12345678;
	make_can_frame(&request, TEST_REQ_ID, 8,
		       ASR_COMM_CMD_READ, ASR_COMM_PARAM_JOINT1);

	int ret = asr_can_protocol_handle_frame(&request, &response,
						&has_response);

	zassert_equal(ret, 0, "joint1 read should succeed");
	zassert_true(has_response, "joint1 read should have response");
	zassert_true(fake.position_read_called, "position callback should be called");
	zassert_equal(response.can_id, TEST_RESP_ID, "response CAN ID should match");
	zassert_equal(response.data[0], 0x78, "position byte 0");
	zassert_equal(response.data[1], 0x56, "position byte 1");
	zassert_equal(response.data[2], 0x34, "position byte 2");
	zassert_equal(response.data[3], 0x12, "position byte 3");
}

ZTEST(can_protocol_transport, test_dlc_must_be_8)
{
	struct asr_can_frame request;
	struct asr_can_frame response;
	bool has_response;

	/* DLC = 4 should be rejected. */
	make_can_frame(&request, TEST_REQ_ID, 4,
		       ASR_COMM_CMD_READ, ASR_COMM_PARAM_IMU);

	int ret = asr_can_protocol_handle_frame(&request, &response,
						&has_response);

	zassert_equal(ret, -EMSGSIZE, "DLC=4 should return -EMSGSIZE");
	zassert_false(has_response, "no response expected");
	zassert_false(fake.imu_read_called, "callback should not be called");
}

ZTEST(can_protocol_transport, test_led_write_no_response)
{
	struct asr_can_frame request;
	struct asr_can_frame response;
	bool has_response;

	/* LED WRITE is a write command -- no response expected. */
	make_can_frame(&request, TEST_REQ_ID, 8,
		       ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_LED_ENABLE);
	request.data[7] = 1U;

	int ret = asr_can_protocol_handle_frame(&request, &response,
						&has_response);

	zassert_equal(ret, 0, "LED write should succeed");
	zassert_false(has_response, "LED write should not have response");
}

ZTEST(can_protocol_transport, test_null_parameters)
{
	bool has_response;
	struct asr_can_frame frame;

	memset(&frame, 0, sizeof(frame));

	int ret = asr_can_protocol_handle_frame(NULL, &frame, &has_response);
	zassert_equal(ret, -EINVAL, "NULL request should return -EINVAL");

	ret = asr_can_protocol_handle_frame(&frame, NULL, &has_response);
	zassert_equal(ret, -EINVAL, "NULL response should return -EINVAL");

	ret = asr_can_protocol_handle_frame(&frame, &frame, NULL);
	zassert_equal(ret, -EINVAL, "NULL has_response should return -EINVAL");
}

/* -------------------------------------------------------------------------
 * Test suite
 * ------------------------------------------------------------------------- */

ZTEST_SUITE(can_protocol_transport, NULL, setup, before_test, NULL, NULL);