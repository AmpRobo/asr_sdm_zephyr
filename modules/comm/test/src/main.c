/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the transport-independent protocol core.
 *
 * Uses fake hardware callbacks to verify that protocol_core_process()
 * correctly dispatches commands, updates unit_status, and returns the
 * expected result object -- without any UART or CAN hardware.
 */

#include <asr/protocol_core.h>

#include <string.h>

#include <zephyr/ztest.h>

/* -------------------------------------------------------------------------
 * Fake callback state
 * ------------------------------------------------------------------------- */

static struct {
	bool flash_write_called;
	const uint8_t *flash_write_data;
	size_t flash_write_len;

	bool led_write_called;
	bool led_write_state;

	bool motor_set_called[2];
	uint8_t motor_value[2];

	bool dynamixel_torque_called;
	uint8_t torque_id;
	bool torque_enable;

	bool dynamixel_goal_position_called;
	uint8_t goal_position_id;
	int32_t goal_position_value;

	bool dynamixel_position_read_called;
	uint8_t position_read_id;
	int32_t position_read_value;

	bool imu_read_called;
	uint8_t imu_read_response[ASR_COMM_MSG_SIZE];
} fake;

static void reset_fake(void)
{
	memset(&fake, 0, sizeof(fake));
}

/* -------------------------------------------------------------------------
 * Fake callbacks
 * ------------------------------------------------------------------------- */

static void fake_on_flash_write(const uint8_t *data, size_t len)
{
	fake.flash_write_called = true;
	fake.flash_write_data = data;
	fake.flash_write_len = len;
}

static void fake_on_led_write(bool state)
{
	fake.led_write_called = true;
	fake.led_write_state = state;
}

static void fake_on_motor_set(uint8_t channel, uint8_t value)
{
	if (channel < ARRAY_SIZE(fake.motor_set_called)) {
		fake.motor_set_called[channel] = true;
		fake.motor_value[channel] = value;
	}
}

static void fake_on_dynamixel_torque(uint8_t id, bool enable)
{
	fake.dynamixel_torque_called = true;
	fake.torque_id = id;
	fake.torque_enable = enable;
}

static void fake_on_dynamixel_goal_position(uint8_t id, int32_t goal_position)
{
	fake.dynamixel_goal_position_called = true;
	fake.goal_position_id = id;
	fake.goal_position_value = goal_position;
}

static int fake_on_dynamixel_position_read(uint8_t id, int32_t *position)
{
	fake.dynamixel_position_read_called = true;
	fake.position_read_id = id;
	*position = fake.position_read_value;
	return 0;
}

static int fake_on_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE])
{
	fake.imu_read_called = true;
	memcpy(buf, fake.imu_read_response, ASR_COMM_MSG_SIZE);
	return 0;
}

static const struct asr_comm_callbacks test_callbacks = {
	.on_flash_write          = fake_on_flash_write,
	.on_led_write            = fake_on_led_write,
	.on_motor_set            = fake_on_motor_set,
	.on_dynamixel_torque     = fake_on_dynamixel_torque,
	.on_dynamixel_goal_position = fake_on_dynamixel_goal_position,
	.on_dynamixel_position_read = fake_on_dynamixel_position_read,
	.on_imu_read             = fake_on_imu_read,
};

/* -------------------------------------------------------------------------
 * Setup / teardown
 * ------------------------------------------------------------------------- */

static void *setup(void)
{
	asr_protocol_core_init();
	asr_protocol_core_register_callbacks(&test_callbacks);
	return NULL;
}

static void before_test(void *fixture)
{
	ARG_UNUSED(fixture);
	reset_fake();
}

/* -------------------------------------------------------------------------
 * Helper: build an 8-byte protocol message
 * ------------------------------------------------------------------------- */

static void build_msg(uint8_t msg[ASR_COMM_MSG_SIZE],
		      uint8_t cmd, uint8_t param,
		      uint32_t value)
{
	memset(msg, 0, ASR_COMM_MSG_SIZE);
	msg[2] = cmd;
	msg[3] = param;
	/* value goes into msg[4..7] little-endian */
	memcpy(&msg[4], &value, sizeof(value));
}

/* -------------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------------- */

ZTEST(asr_protocol_core, test_imu_read_returns_response)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	/* Set up fake IMU response. */
	fake.imu_read_response[0] = 0xAA;
	fake.imu_read_response[1] = 0xBB;
	fake.imu_read_response[2] = 0xCC;
	fake.imu_read_response[3] = 0xDD;

	build_msg(request, ASR_COMM_CMD_READ, ASR_COMM_PARAM_IMU, 0);

	int ret = asr_protocol_core_process(request, &result);

	zassert_equal(ret, 0, "IMU read should return 0");
	zassert_true(result.has_response, "IMU read should have response");
	zassert_true(fake.imu_read_called, "on_imu_read should be called");
	zassert_equal(result.response[0], 0xAA, "response[0] mismatch");
	zassert_equal(result.response[1], 0xBB, "response[1] mismatch");
	zassert_equal(result.response[2], 0xCC, "response[2] mismatch");
	zassert_equal(result.response[3], 0xDD, "response[3] mismatch");
}

ZTEST(asr_protocol_core, test_imu_read_no_callback)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	/* Unregister callbacks. */
	asr_protocol_core_register_callbacks(NULL);

	build_msg(request, ASR_COMM_CMD_READ, ASR_COMM_PARAM_IMU, 0);

	int ret = asr_protocol_core_process(request, &result);

	zassert_equal(ret, -ENOSYS, "IMU read without callback should return -ENOSYS");
	zassert_false(result.has_response, "no response expected");

	/* Restore callbacks for subsequent tests. */
	asr_protocol_core_register_callbacks(&test_callbacks);
}

ZTEST(asr_protocol_core, test_led_enable_1)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_LED_ENABLE, 0);
	request[7] = 1U; /* enable = 1 */

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "LED write should not have response");
	zassert_true(asr_protocol_core_get_status()->led_enable,
		     "led_enable should be true");
	zassert_false(fake.led_write_called,
		      "on_led_write should not be called for enable=1");
}

ZTEST(asr_protocol_core, test_led_enable_0)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_LED_ENABLE, 0);
	request[7] = 0U; /* enable = 0 */

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "LED write should not have response");
	zassert_false(asr_protocol_core_get_status()->led_enable,
		      "led_enable should be false");
	zassert_true(fake.led_write_called, "on_led_write should be called");
	zassert_true(fake.led_write_state, "on_led_write should receive true");
}

ZTEST(asr_protocol_core, test_led_enable_invalid_value)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_LED_ENABLE, 0);
	request[7] = 2U; /* invalid */

	int ret = asr_protocol_core_process(request, &result);

	zassert_equal(ret, -EINVAL, "invalid LED value should return -EINVAL");
	zassert_false(result.has_response, "no response expected");
}

ZTEST(asr_protocol_core, test_motor_set)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_MOTOR, 0);
	request[6] = 0xAB;
	request[7] = 0xCD;

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "motor write should not have response");
	zassert_true(fake.motor_set_called[0], "on_motor_set channel 0");
	zassert_true(fake.motor_set_called[1], "on_motor_set channel 1");
	zassert_equal(fake.motor_value[0], 0xAB, "motor channel 0 value");
	zassert_equal(fake.motor_value[1], 0xCD, "motor channel 1 value");
	/* Also check stored status. */
	zassert_equal(asr_protocol_core_get_status()->cmd_motor[0], 0xAB, "status motor[0]");
	zassert_equal(asr_protocol_core_get_status()->cmd_motor[1], 0xCD, "status motor[1]");
}

ZTEST(asr_protocol_core, test_joint1_position)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	/* 0x12345678 in little-endian */
	uint32_t position = 0x12345678;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_JOINT1, position);

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "joint1 write should not have response");
	zassert_true(fake.dynamixel_goal_position_called,
		     "on_dynamixel_goal_position should be called");
	zassert_equal(fake.goal_position_id, ASR_DXL_1, "should be DXL_1");
	zassert_equal(fake.goal_position_value, (int32_t)0x12345678,
		      "position value mismatch");
}

ZTEST(asr_protocol_core, test_joint2_position)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	/* Negative position in little-endian */
	int32_t position = -1000;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_JOINT2,
		  (uint32_t)position);

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "joint2 write should not have response");
	zassert_true(fake.dynamixel_goal_position_called,
		     "on_dynamixel_goal_position should be called");
	zassert_equal(fake.goal_position_id, ASR_DXL_2, "should be DXL_2");
	zassert_equal(fake.goal_position_value, -1000,
		      "negative position value mismatch");
}

ZTEST(asr_protocol_core, test_joint1_position_read_returns_response)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	fake.position_read_value = 0x12345678;
	build_msg(request, ASR_COMM_CMD_READ, ASR_COMM_PARAM_JOINT1, 0);

	int ret = asr_protocol_core_process(request, &result);

	zassert_equal(ret, 0, "joint1 read should return 0");
	zassert_true(result.has_response, "joint1 read should have response");
	zassert_true(fake.dynamixel_position_read_called,
		     "on_dynamixel_position_read should be called");
	zassert_equal(fake.position_read_id, ASR_DXL_1, "should be DXL_1");
	zassert_equal(result.response[0], 0x78, "response[0]");
	zassert_equal(result.response[1], 0x56, "response[1]");
	zassert_equal(result.response[2], 0x34, "response[2]");
	zassert_equal(result.response[3], 0x12, "response[3]");
	zassert_equal(result.response[4], 0x00, "response[4]");
}

ZTEST(asr_protocol_core, test_joint1_torque_enable)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_JOINT1_TORQUE, 0);
	request[4] = 1U;

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "torque write should not have response");
	zassert_true(fake.dynamixel_torque_called,
		     "on_dynamixel_torque should be called");
	zassert_equal(fake.torque_id, ASR_DXL_1, "should be DXL_1");
	zassert_true(fake.torque_enable, "torque should be enabled");
	zassert_true(asr_protocol_core_get_status()->dynamixel_enable[ASR_DXL_1],
		     "status torque enable");
}

ZTEST(asr_protocol_core, test_joint1_torque_disable)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_JOINT1_TORQUE, 0);
	request[4] = 0U;

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "torque write should not have response");
	zassert_true(fake.dynamixel_torque_called,
		     "on_dynamixel_torque should be called");
	zassert_false(fake.torque_enable, "torque should be disabled");
}

ZTEST(asr_protocol_core, test_joint1_torque_invalid)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_JOINT1_TORQUE, 0);
	request[4] = 2U; /* invalid */

	int ret = asr_protocol_core_process(request, &result);

	zassert_equal(ret, -EINVAL, "invalid torque value should return -EINVAL");
	zassert_false(result.has_response, "no response expected");
}

ZTEST(asr_protocol_core, test_unknown_command)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, 0xFF, ASR_COMM_PARAM_LED_ENABLE, 0);

	int ret = asr_protocol_core_process(request, &result);

	zassert_equal(ret, -ENOTSUP, "unknown cmd should return -ENOTSUP");
	zassert_false(result.has_response, "no response expected");
}

ZTEST(asr_protocol_core, test_dynamixel_bulk_rejected)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	/* 0xFF 0xFD is the Dynamixel bulk-read marker. */
	memset(request, 0, ASR_COMM_MSG_SIZE);
	request[0] = 0xFF;
	request[1] = 0xFD;

	int ret = asr_protocol_core_process(request, &result);

	zassert_equal(ret, -EINVAL, "Dynamixel bulk frame should return -EINVAL");
	zassert_false(result.has_response, "no response expected");
}

ZTEST(asr_protocol_core, test_imu_read_result_zeroed_on_entry)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	/* Fill result with junk to verify it gets zeroed. */
	memset(&result, 0xFF, sizeof(result));

	/* Send an unknown command - should zero result. */
	build_msg(request, ASR_COMM_CMD_READ, 0xFF, 0);

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response,
		      "result should be zeroed (has_response=false)");
}

ZTEST(asr_protocol_core, test_null_parameters)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	memset(request, 0, ASR_COMM_MSG_SIZE);

	/* NULL request. */
	int ret = asr_protocol_core_process(NULL, &result);
	zassert_equal(ret, -EINVAL, "NULL request should return -EINVAL");

	/* NULL result. */
	ret = asr_protocol_core_process(request, NULL);
	zassert_equal(ret, -EINVAL, "NULL result should return -EINVAL");
}

ZTEST(asr_protocol_core, test_can_id_write)
{
	uint8_t request[ASR_COMM_MSG_SIZE];
	struct asr_protocol_result result;

	build_msg(request, ASR_COMM_CMD_WRITE, ASR_COMM_PARAM_CAN_ID, 0);
	request[6] = 0x01;
	request[7] = 0x23;

	asr_protocol_core_process(request, &result);

	zassert_false(result.has_response, "CAN_ID write should not have response");
	zassert_true(fake.flash_write_called, "on_flash_write should be called");
	zassert_equal(asr_protocol_core_get_status()->flash_data[0], 0x01,
		      "flash_data[0]");
	zassert_equal(asr_protocol_core_get_status()->flash_data[1], 0x23,
		      "flash_data[1]");
}

/* -------------------------------------------------------------------------
 * Test suite
 * ------------------------------------------------------------------------- */

ZTEST_SUITE(asr_protocol_core, NULL, setup, before_test, NULL, NULL);