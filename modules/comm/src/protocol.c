/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Protocol handler and UART transport for the ASR SDM communication module.
 *
 * Interrupt-driven RX reassembles framed messages through a byte-level state
 * machine and enqueues complete payloads for the processing thread.  TX uses
 * blocking poll-out (suitable for low-frequency command responses).
 *
 * Wire frame:
 *   [0xAA] [0x55] [LEN] [DATA_0 .. DATA_N-1] [CHK]
 *   CHK = LEN ^ DATA_0 ^ ... ^ DATA_N-1
 */

#include <asr/comm_thread.h>
#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_comm, LOG_LEVEL_INF);

#define COMM_UART_NODE DT_NODELABEL(uart0)

#define FRAME_SYNC_HI 0xAAU
#define FRAME_SYNC_LO 0x55U
#define FRAME_MAX_PAYLOAD ASR_COMM_MSG_SIZE

BUILD_ASSERT(DT_NODE_EXISTS(COMM_UART_NODE), "uart0 node must exist in devicetree");

/* --- UART device --------------------------------------------------------- */

static const struct device *const uart_dev = DEVICE_DT_GET(COMM_UART_NODE);

/* --- RX state machine ---------------------------------------------------- */

enum rx_state
{
    RX_SYNC1,
    RX_SYNC2,
    RX_LENGTH,
    RX_DATA,
    RX_CHECKSUM,
};

K_MSGQ_DEFINE(rx_msgq, ASR_COMM_MSG_SIZE, CONFIG_ASR_COMM_RX_QUEUE_DEPTH, 4);
K_SEM_DEFINE(rx_frame_sem, 0, CONFIG_ASR_COMM_RX_QUEUE_DEPTH);

static struct
{
    enum rx_state state;
    uint8_t buf[FRAME_MAX_PAYLOAD];
    uint8_t expected_len;
    uint8_t idx;
    uint8_t checksum;
} rx_ctx;

static void rx_process_byte(uint8_t byte)
{
    switch (rx_ctx.state)
    {
    case RX_SYNC1:
        if (byte == FRAME_SYNC_HI)
        {
            rx_ctx.state = RX_SYNC2;
        }
        break;

    case RX_SYNC2:
        rx_ctx.state = (byte == FRAME_SYNC_LO) ? RX_LENGTH : RX_SYNC1;
        break;

    case RX_LENGTH:
        if (byte == 0U || byte > FRAME_MAX_PAYLOAD)
        {
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
        if (rx_ctx.idx >= rx_ctx.expected_len)
        {
            rx_ctx.state = RX_CHECKSUM;
        }
        break;

    case RX_CHECKSUM:
        if (byte == rx_ctx.checksum)
        {
            if (rx_ctx.expected_len < ASR_COMM_MSG_SIZE)
            {
                memset(&rx_ctx.buf[rx_ctx.expected_len], 0, ASR_COMM_MSG_SIZE - rx_ctx.expected_len);
            }
            k_msgq_put(&rx_msgq, rx_ctx.buf, K_NO_WAIT);
            k_sem_give(&rx_frame_sem);
        }
        else
        {
            LOG_WRN("frame checksum mismatch");
        }
        rx_ctx.state = RX_SYNC1;
        break;
    }
}

static void uart_isr_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev))
    {
        if (!uart_irq_rx_ready(dev))
        {
            continue;
        }
        uint8_t byte;

        while (uart_fifo_read(dev, &byte, 1) == 1)
        {
            rx_process_byte(byte);
        }
    }
}

/* --- protocol state ------------------------------------------------------ */

static asr_unit_status_t unit_status;
static const struct asr_comm_callbacks *hw_cb;

void asr_comm_register_callbacks(const struct asr_comm_callbacks *cb) { hw_cb = cb; }

const asr_unit_status_t *asr_comm_get_status(void) { return &unit_status; }

/* --- public UART / protocol API ------------------------------------------ */

int asr_comm_uart_init(void)
{
    if (!device_is_ready(uart_dev))
    {
        LOG_ERR("UART0 device not ready");
        return -ENODEV;
    }

    uart_irq_callback_set(uart_dev, uart_isr_cb);
    uart_irq_rx_enable(uart_dev);

    LOG_INF("UART0 communication module initialised");
    return 0;
}

int asr_comm_send(const uint8_t data[ASR_COMM_MSG_SIZE])
{
    if (!device_is_ready(uart_dev))
    {
        return -ENODEV;
    }

    uint8_t checksum = ASR_COMM_MSG_SIZE;

    uart_poll_out(uart_dev, FRAME_SYNC_HI);
    uart_poll_out(uart_dev, FRAME_SYNC_LO);
    uart_poll_out(uart_dev, ASR_COMM_MSG_SIZE);

    for (int i = 0; i < ASR_COMM_MSG_SIZE; i++)
    {
        uart_poll_out(uart_dev, data[i]);
        checksum ^= data[i];
    }

    uart_poll_out(uart_dev, checksum);
    return 0;
}

bool protocol_init(void)
{
    unit_status.unit_id = (uint32_t)0x00000000;

    return true;
}

bool protocol_update(const uint8_t msg[ASR_COMM_MSG_SIZE])
{
    if (msg[0] == 0xFF && msg[1] == 0xFD)
    {
        return false;
    }

    if (msg[2] == ASR_COMM_CMD_READ)
    {
        switch (msg[3])
        {
        case ASR_COMM_PARAM_BOARD_ID:
        case ASR_COMM_PARAM_CAN_ID:
        case ASR_COMM_PARAM_LED_ENABLE:
        case ASR_COMM_PARAM_LED_STATUS:
        case ASR_COMM_PARAM_MOTOR:
        case ASR_COMM_PARAM_JOINT1:
        case ASR_COMM_PARAM_JOINT2:
        case ASR_COMM_PARAM_JOINT1_TORQUE:
        case ASR_COMM_PARAM_JOINT2_TORQUE:
            break;
        case ASR_COMM_PARAM_IMU:
        {
            uint8_t reply[ASR_COMM_MSG_SIZE] = {0};

            if (hw_cb && hw_cb->on_imu_read && hw_cb->on_imu_read(reply) == 0)
            {
                asr_comm_send(reply);
            }
            break;
        }
        default:
            break;
        }
    }
    else if (msg[2] == ASR_COMM_CMD_WRITE)
    {
        switch (msg[3])
        {
        case ASR_COMM_PARAM_CAN_ID:
            unit_status.flash_data[0] = msg[6];
            unit_status.flash_data[1] = msg[7];
            if (hw_cb && hw_cb->on_flash_write)
            {
                hw_cb->on_flash_write(unit_status.flash_data, sizeof(unit_status.flash_data));
            }
            break;

        case ASR_COMM_PARAM_LED_ENABLE:
            if (msg[7] == 1)
            {
                unit_status.led_enable = true;
            }
            else if (msg[7] == 0)
            {
                unit_status.led_enable = false;
                unit_status.led_status = true;
                if (hw_cb && hw_cb->on_led_write)
                {
                    hw_cb->on_led_write(true);
                }
            }
            break;

        case ASR_COMM_PARAM_LED_STATUS:
            break;

        case ASR_COMM_PARAM_MOTOR:
            unit_status.cmd_motor[0] = msg[6];
            unit_status.cmd_motor[1] = msg[7];
            if (hw_cb && hw_cb->on_motor_set)
            {
                hw_cb->on_motor_set(0, unit_status.cmd_motor[0]);
                hw_cb->on_motor_set(1, unit_status.cmd_motor[1]);
            }
            break;

		case ASR_COMM_PARAM_JOINT1:
			unit_status.cmd_joint1[0] = msg[4];
			unit_status.cmd_joint1[1] = msg[5];
			unit_status.cmd_joint1[2] = msg[6];
			unit_status.cmd_joint1[3] = msg[7];
			if (hw_cb && hw_cb->on_dynamixel_goal_position) {
				hw_cb->on_dynamixel_goal_position(
					ASR_DXL_1,
					(int32_t)sys_get_le32(&msg[4]));
			}
			break;

		case ASR_COMM_PARAM_JOINT2:
			unit_status.cmd_joint2[0] = msg[4];
			unit_status.cmd_joint2[1] = msg[5];
			unit_status.cmd_joint2[2] = msg[6];
			unit_status.cmd_joint2[3] = msg[7];
			if (hw_cb && hw_cb->on_dynamixel_goal_position) {
				hw_cb->on_dynamixel_goal_position(
					ASR_DXL_2,
					(int32_t)sys_get_le32(&msg[4]));
			}
			break;

        case ASR_COMM_PARAM_JOINT1_TORQUE:
            unit_status.dynamixel_enable[ASR_DXL_1] = (bool)msg[4];
            if (hw_cb && hw_cb->on_dynamixel_torque)
            {
                hw_cb->on_dynamixel_torque(ASR_DXL_1, unit_status.dynamixel_enable[ASR_DXL_1]);
            }
            break;

        case ASR_COMM_PARAM_JOINT2_TORQUE:
            unit_status.dynamixel_enable[ASR_DXL_2] = (bool)msg[4];
            if (hw_cb && hw_cb->on_dynamixel_torque)
            {
                hw_cb->on_dynamixel_torque(ASR_DXL_2, unit_status.dynamixel_enable[ASR_DXL_2]);
            }
            break;

        default:
            break;
        }
    }

    return true;
}
