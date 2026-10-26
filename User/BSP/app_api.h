#ifndef __APP_API_H__
#define __APP_API_H__

#include "stdint.h"
#include "BoardLink.h"

/* ---- 任务级入口（由 freertos.c 各 Task 调用） ---- */
void App_RemoteStep(void);
void App_GimbalInit(void);
void App_GimbalStep(void);
void App_ChassisInit(void);
void App_ChassisStep(void);
void App_LinkStep(void);
void App_MotorStep(void);
int  App_WatchdogRefreshAllowed(void);
void App_ShootInit(void);
void App_ShootStep(void);
void App_RefereeStep(void);

/* ---- USB CDC 收到视觉数据回调 ---- */
void App_OnVisionBytes(const uint8_t *buf, uint16_t len);
void App_OnUsbFrame(const uint8_t *data, uint16_t len);

#endif
