#include "cmsis_os2.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "telemetry.h"
#include "usart.h"
#include "dma.h"

#define BT24_RX_RING_SIZE 128U
#define BT24_RX_DMA_BUF_SIZE 64U

extern DMA_HandleTypeDef hdma_usart2_rx;

static uint8_t rx_ring[BT24_RX_RING_SIZE];
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;
static uint8_t rx_dma_buf[BT24_RX_DMA_BUF_SIZE];

static void bt24_rx_push_byte(uint8_t ch)
{
    uint16_t next_head = (uint16_t)((rx_head + 1U) % BT24_RX_RING_SIZE);
    if (next_head == rx_tail) {
        /* Drop oldest byte when full to keep latest stream data. */
        rx_tail = (uint16_t)((rx_tail + 1U) % BT24_RX_RING_SIZE);
    }
    rx_ring[rx_head] = ch;
    rx_head = next_head;
}

static uint8_t bt24_rx_pop_byte(uint8_t *ch)
{
    if (rx_tail == rx_head) {
        return 0U;
    }
    *ch = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % BT24_RX_RING_SIZE);
    return 1U;
}

static void queue_bt_cmd(const BtCmd_t *cmd)
{
    BtCmd_t old_cmd;

    if (osMessageQueuePut(BtCmdQueueHandle, cmd, 0U, 0U) != osOK) {
        (void)osMessageQueueGet(BtCmdQueueHandle, &old_cmd, NULL, 0U);
        (void)osMessageQueuePut(BtCmdQueueHandle, cmd, 0U, 0U);
    }
}

static uint8_t parse_bt_line(const char *line, BtCmd_t *cmd)
{
    char action;
    long p1 = 0;
    long p2 = 0;
    long p3 = 0;
    int count = 0;
    char temp[24];
    char *token;
    char *ctx = NULL;

    while ((*line == ' ') || (*line == '\t')) {
        line++;
    }
    if (*line == '\0') {
        return 0U;
    }

    action = (char)toupper((unsigned char)*line);
    if ((action != 'L') && (action != 'R') && (action != 'S') && (action != 'B') && (action != 'X')) {
        return 0U;
    }

    cmd->action = (uint8_t)action;
    cmd->angle_deg = 0;
    cmd->speed_pct = 0U;
    cmd->duration_ms = 0U;

    if (action == 'X') {
        return 1U;
    }

    strncpy(temp, line, sizeof(temp) - 1U);
    temp[sizeof(temp) - 1U] = '\0';
    token = strtok_r(temp, ",", &ctx);
    while (token != NULL) {
        while ((*token == ' ') || (*token == '\t')) {
            token++;
        }
        if (count == 1) {
            p1 = strtol(token, NULL, 10);
        } else if (count == 2) {
            p2 = strtol(token, NULL, 10);
        } else if (count == 3) {
            p3 = strtol(token, NULL, 10);
        }
        count++;
        token = strtok_r(NULL, ",", &ctx);
    }

    if ((action == 'S') || (action == 'B')) {
        if (count < 3) {
            return 0U;
        }
        if (p1 < 0) p1 = 0;
        if (p1 > 100) p1 = 100;
        if (p2 < 0) p2 = 0;
        if (p2 > 600) p2 = 600;

        cmd->speed_pct = (uint8_t)p1;
        cmd->duration_ms = (uint32_t)p2 * 1000U;
    } else {
        if (count < 4) {
            return 0U;
        }
        if (p1 < 0) p1 = -p1;
        if (p1 > 180) p1 = 180;
        if (p2 < 0) p2 = 0;
        if (p2 > 100) p2 = 100;
        if (p3 < 0) p3 = 0;
        if (p3 > 600) p3 = 600;

        cmd->angle_deg = (int16_t)p1;
        cmd->speed_pct = (uint8_t)p2;
        cmd->duration_ms = (uint32_t)p3 * 1000U;
    }

    return 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        uint16_t i;
        for (i = 0U; i < Size; i++) {
            bt24_rx_push_byte(rx_dma_buf[i]);
        }
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, BT24_RX_DMA_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, BT24_RX_DMA_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

void StartBT24Task(void *argument)
{
    uint8_t ch;
    uint8_t idx = 0U;
    uint32_t last_rx_tick = 0U;
    char line_buf[24];
    BtCmd_t cmd;

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, BT24_RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    for (;;) {
        uint8_t had_data = 0U;

        while (bt24_rx_pop_byte(&ch) != 0U) {
            had_data = 1U;
            last_rx_tick = osKernelGetTickCount();
            if ((ch == '\r') || (ch == '\n')) {
                if (idx > 0U) {
                    line_buf[idx] = '\0';
                    if (parse_bt_line(line_buf, &cmd) != 0U) {
                        queue_bt_cmd(&cmd);
                    }
                    idx = 0U;
                }
            } else if (idx < (sizeof(line_buf) - 1U)) {
                line_buf[idx++] = (char)ch;
            } else {
                idx = 0U;
            }
        }

        if ((idx > 0U) && ((osKernelGetTickCount() - last_rx_tick) > 120U)) {
            line_buf[idx] = '\0';
            if (parse_bt_line(line_buf, &cmd) != 0U) {
                queue_bt_cmd(&cmd);
            }
            idx = 0U;
        }

        if (had_data == 0U) {
            osDelay(5U);
        }
    }
}
