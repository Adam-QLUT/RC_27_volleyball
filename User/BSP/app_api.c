#include "app_api.h"

/* 桩实现：待业务逻辑补全 */

BoardLink_t BoardLink = {0.0f, 0.0f};

void App_RemoteStep(void)        {}
void App_GimbalInit(void)        {}
void App_GimbalStep(void)        {}
void App_ChassisInit(void)       {}
void App_ChassisStep(void)       {}
void App_LinkStep(void)          {}
void App_MotorStep(void)         {}
int  App_WatchdogRefreshAllowed(void) { return 1; }
void App_ShootInit(void)         {}
void App_ShootStep(void)         {}
void App_RefereeStep(void)       {}
void App_OnVisionBytes(const uint8_t *buf, uint16_t len) { (void)buf; (void)len; }
void App_OnUsbFrame(const uint8_t *data, uint16_t len)    { (void)data; (void)len; }

int BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, const uint8_t *data)
{
    (void)hfdcan; (void)id; (void)data;
    return 0;
}

int BoardLink_UartRxDispatch(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size)
{
    (void)huart; (void)data; (void)size;
    return 0;
}
void BoardLink_UartTxCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }
void BoardLink_UartErrorCallback(UART_HandleTypeDef *huart)  { (void)huart; }
