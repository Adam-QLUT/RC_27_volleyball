#ifndef __BOARDLINK_H__
#define __BOARDLINK_H__

#include "stdint.h"
#include "stm32h7xx_hal.h"

/* Board-link CAN 命令 ID 示例 */
#define BL_ID_YAW_SPEED_CMD   0x1FFU

/* Board-link (视觉/上位机) 数据 */
typedef struct
{
    float vx;
    float vy;
} BoardLink_t;

extern BoardLink_t BoardLink;

/* FDCAN 接收分发：返回是否消费了该帧 */
int BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, const uint8_t *data);

/* UART 接收分发：返回是否消费了该帧 */
int BoardLink_UartRxDispatch(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);
void BoardLink_UartTxCpltCallback(UART_HandleTypeDef *huart);
void BoardLink_UartErrorCallback(UART_HandleTypeDef *huart);

#endif
