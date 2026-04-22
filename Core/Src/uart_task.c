#include "uart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "system_globals.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdint.h>

extern UART_HandleTypeDef huart5;

#define UART_FRAME_SIZE            12U

#define UART_RX_START              0xAA
#define UART_RX_END                0x0A

#define UART_TX_START_1            0xAA
#define UART_TX_START_2            0xBB
#define UART_TX_END                0x0A

#define UART_TELEMETRY_PERIOD_MS   20U
#define UART_CMD_TIMEOUT_MS        100U

static uint8_t uart_rx_byte = 0;

static uint8_t rx_work_buf[UART_FRAME_SIZE];
static uint8_t rx_frame_buf[UART_FRAME_SIZE];

static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_frame_ready = 0;

static uint32_t last_valid_cmd_tick = 0;

static uint8_t UART_ChecksumSum(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFF);
}

static int16_t UART_ReadInt16LE(const uint8_t *p)
{
    uint16_t raw = ((uint16_t)p[0]) | ((uint16_t)p[1] << 8);
    return (int16_t)raw;
}

static void UART_WriteUInt16LE(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void UART_StartReception(void)
{
    HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
}

static void UART_StopRobot(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < 4; i++)
    {
        speed[i] = 0.0f;
        manual_dac_cmd[i] = 0;
    }
    current_mode = UART_MODE_CLOSED_LOOP;
    taskEXIT_CRITICAL();
}

static uint8_t UART_IsValidRxFrame(const uint8_t *frame)
{
    if (frame[0] != UART_RX_START) return 0;
    if (frame[11] != UART_RX_END) return 0;

    if (UART_ChecksumSum(&frame[1], 9) != frame[10]) return 0;

    return 1;
}

static void UART_ProcessRxFrame(const uint8_t *frame)
{
    uint8_t mode;
    int16_t val_m1, val_m2, val_m3, val_m4;

    if (!UART_IsValidRxFrame(frame))
    {
        return;
    }

    mode   = frame[1];
    val_m1 = UART_ReadInt16LE(&frame[2]);
    val_m2 = UART_ReadInt16LE(&frame[4]);
    val_m3 = UART_ReadInt16LE(&frame[6]);
    val_m4 = UART_ReadInt16LE(&frame[8]);

    taskENTER_CRITICAL();

    switch (mode)
    {
        case UART_MODE_CLOSED_LOOP:
            speed[0] = ((float)val_m1) / 100.0f;
            speed[1] = ((float)val_m2) / 100.0f;
            speed[2] = ((float)val_m3) / 100.0f;
            speed[3] = ((float)val_m4) / 100.0f;
            current_mode = mode;
            break;

        case UART_MODE_MANUAL_DAC:
            manual_dac_cmd[0] = val_m1;
            manual_dac_cmd[1] = val_m2;
            manual_dac_cmd[2] = val_m3;
            manual_dac_cmd[3] = val_m4;
            current_mode = mode;
            break;

        default:
            for (uint8_t i = 0; i < 4; i++)
            {
                speed[i] = 0.0f;
                manual_dac_cmd[i] = 0;
            }
            current_mode = UART_MODE_CLOSED_LOOP;
            break;
    }

    taskEXIT_CRITICAL();

    last_valid_cmd_tick = osKernelSysTick();
}

static void UART_SendTelemetryFrame(void)
{
    uint8_t tx[UART_FRAME_SIZE];
    uint16_t enc1, enc2, enc3, enc4;

    enc1 = (uint16_t)(*motor[0].encoder.count);
    enc2 = (uint16_t)(*motor[1].encoder.count);
    enc3 = (uint16_t)(*motor[2].encoder.count);
    enc4 = (uint16_t)(*motor[3].encoder.count);

    tx[0] = UART_TX_START_1;
    tx[1] = UART_TX_START_2;

    UART_WriteUInt16LE(&tx[2], enc1);
    UART_WriteUInt16LE(&tx[4], enc2);
    UART_WriteUInt16LE(&tx[6], enc3);
    UART_WriteUInt16LE(&tx[8], enc4);

    tx[10] = UART_ChecksumSum(&tx[2], 8);
    tx[11] = UART_TX_END;

    HAL_UART_Transmit(&huart5, tx, UART_FRAME_SIZE, 10);
}

void UARTTaskFunction(void const *argument)
{
    uint32_t timeToWait;
    uint8_t local_frame[UART_FRAME_SIZE];

    (void)argument;

    timeToWait = osKernelSysTick();
    last_valid_cmd_tick = osKernelSysTick();

    rx_index = 0;
    rx_frame_ready = 0;
    memset(rx_work_buf, 0, sizeof(rx_work_buf));
    memset(rx_frame_buf, 0, sizeof(rx_frame_buf));

    UART_StartReception();

    for (;;)
    {
        if (rx_frame_ready)
        {
            taskENTER_CRITICAL();
            memcpy(local_frame, rx_frame_buf, UART_FRAME_SIZE);
            rx_frame_ready = 0;
            taskEXIT_CRITICAL();

            UART_ProcessRxFrame(local_frame);
        }

        if ((osKernelSysTick() - last_valid_cmd_tick) > UART_CMD_TIMEOUT_MS)
        {
            UART_StopRobot();
        }

        UART_SendTelemetryFrame();
        osDelayUntil(&timeToWait, UART_TELEMETRY_PERIOD_MS);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        uint8_t b = uart_rx_byte;

        if (rx_index == 0)
        {
            if (b == UART_RX_START)
            {
                rx_work_buf[0] = b;
                rx_index = 1;
            }
        }
        else
        {
            rx_work_buf[rx_index++] = b;

            if (rx_index >= UART_FRAME_SIZE)
            {
                if (!rx_frame_ready)
                {
                    memcpy(rx_frame_buf, rx_work_buf, UART_FRAME_SIZE);
                    rx_frame_ready = 1;
                }
                rx_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
    }
}
