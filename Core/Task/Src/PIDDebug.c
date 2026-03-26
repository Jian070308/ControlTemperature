#include "PIDDebug.h"

#include <string.h>

#include "main.h"
#include "PID.h"
#include "usart.h"

extern uint8_t show_target;

#define PID_DEBUG_FRAME_HEAD 0xC5U
#define PID_DEBUG_FRAME_TAIL 0x5CU
#define PID_DEBUG_MAX_DATA_LEN 32U
#define PID_DEBUG_MAX_FRAME_LEN (1U + 1U + PID_DEBUG_MAX_DATA_LEN + 2U + 1U)

/*
 * These category codes are inferred from the public ATK tutorial's API order:
 * TYPE_PID1..TYPE_PID10, TYPE_AMP, TYPE_VBUS, TYPE_POWER, TYPE_SPEED,
 * TYPE_SUM_LEN, TYPE_TEMP, TYPE_STATUS, TYPE_MOTOR_CODE, TYPE_USER_DATA,
 * TYPE_CTRL_CODE.
 * If your local protocol PDF defines different values, update them here.
 */
#define TYPE_PID1        0x01U
#define TYPE_STATUS      0x11U
#define TYPE_MOTOR_CODE  0x12U
#define TYPE_USER_DATA   0x13U
#define TYPE_CTRL_CODE   0x14U

#define MOTOR_CODE_TEMP_CONTROL 0x01U
#define MOTOR_STATE_IDLE        0x00U

typedef struct
{
    float setpoint;
    float kp;
    float ki;
    float kd;
} pid_frame_t;

typedef struct
{
    int16_t ch[16];
} wave_frame_t;

static uint8_t rx_frame[PID_DEBUG_MAX_FRAME_LEN];
static uint8_t rx_length = 0;
static uint32_t last_wave_tick = 0;
static uint8_t ctrl_code = 0;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i = 0;

    for (i = 0; i < length; ++i)
    {
        uint8_t j = 0;
        crc ^= data[i];
        for (j = 0; j < 8U; ++j)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static void send_frame(uint8_t type, const void *payload, uint8_t payload_len)
{
    uint8_t frame[PID_DEBUG_MAX_FRAME_LEN];
    uint16_t crc = 0;
    uint8_t frame_len = (uint8_t)(payload_len + 5U);

    frame[0] = PID_DEBUG_FRAME_HEAD;
    frame[1] = type;
    if ((payload != NULL) && (payload_len > 0U))
    {
        memcpy(&frame[2], payload, payload_len);
    }
    crc = crc16_modbus(frame, (uint16_t)(payload_len + 2U));
    frame[2U + payload_len] = (uint8_t)(crc & 0xFFU);
    frame[3U + payload_len] = (uint8_t)((crc >> 8U) & 0xFFU);
    frame[4U + payload_len] = PID_DEBUG_FRAME_TAIL;

    USART2_Write(frame, frame_len);
}

static void send_pid_init(void)
{
    pid_frame_t frame;

    frame.setpoint = targetTemp;
    frame.kp = pid.Kp;
    frame.ki = pid.Ki;
    frame.kd = pid.Kd;
    send_frame(TYPE_PID1, &frame, sizeof(frame));
}

static void send_motor_state(uint8_t state)
{
    send_frame(TYPE_STATUS, &state, 1U);
}

static void send_motor_code(uint8_t code)
{
    send_frame(TYPE_MOTOR_CODE, &code, 1U);
}

static void send_wave_frame(void)
{
    wave_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.ch[0] = (int16_t)realTemp;
    frame.ch[1] = (int16_t)targetTemp;
    frame.ch[2] = (int16_t)pid.output;
    frame.ch[3] = (int16_t)(pid.error * 10.0f);

    send_frame(TYPE_USER_DATA, &frame, sizeof(frame));
}

static void handle_pid_frame(const uint8_t *payload, uint8_t payload_len)
{
    pid_frame_t frame;

    if (payload_len < sizeof(frame))
    {
        return;
    }

    memcpy(&frame, payload, sizeof(frame));
    targetTemp = frame.setpoint;
    if (targetTemp < 40.0f)
    {
        targetTemp = 40.0f;
    }
    if (targetTemp > 70.0f)
    {
        targetTemp = 70.0f;
    }
    show_target = (uint8_t)(targetTemp + 0.5f);

    PID_SetTunings(frame.kp, frame.ki, frame.kd);
    PID_Reset();
}

static void handle_ctrl_frame(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len < 1U)
    {
        return;
    }

    ctrl_code = payload[0];
}

static void parse_frame(const uint8_t *frame, uint8_t length)
{
    const uint8_t payload_len = (uint8_t)(length - 5U);
    const uint8_t *payload = &frame[2];
    const uint16_t recv_crc = (uint16_t)(frame[length - 3U] | ((uint16_t)frame[length - 2U] << 8U));
    const uint16_t calc_crc = crc16_modbus(frame, (uint16_t)(length - 3U));

    if ((frame[0] != PID_DEBUG_FRAME_HEAD) || (frame[length - 1U] != PID_DEBUG_FRAME_TAIL))
    {
        return;
    }

    if (recv_crc != calc_crc)
    {
        return;
    }

    switch (frame[1])
    {
        case TYPE_PID1:
            handle_pid_frame(payload, payload_len);
            break;

        case TYPE_CTRL_CODE:
            handle_ctrl_frame(payload, payload_len);
            break;

        default:
            break;
    }
}

void PID_Debug_Init(void)
{
    rx_length = 0;
    ctrl_code = 0;
    last_wave_tick = HAL_GetTick();

    send_motor_code(MOTOR_CODE_TEMP_CONTROL);
    send_motor_state(MOTOR_STATE_IDLE);
    send_pid_init();
}

void PID_Debug_RxCpltCallback(const uint8_t byte)
{
    if (rx_length == 0U)
    {
        if (byte == PID_DEBUG_FRAME_HEAD)
        {
            rx_frame[rx_length++] = byte;
        }
        return;
    }

    if (rx_length < PID_DEBUG_MAX_FRAME_LEN)
    {
        rx_frame[rx_length++] = byte;
    }
    else
    {
        rx_length = 0U;
        return;
    }

    if (byte == PID_DEBUG_FRAME_TAIL)
    {
        parse_frame(rx_frame, rx_length);
        rx_length = 0U;
    }
}

void PID_Debug_Process(void)
{
    if ((HAL_GetTick() - last_wave_tick) >= 100U)
    {
        last_wave_tick = HAL_GetTick();
        send_wave_frame();
    }

    if (ctrl_code != 0U)
    {
        ctrl_code = 0U;
    }
}
