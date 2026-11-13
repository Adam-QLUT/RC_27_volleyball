#ifndef _SHOOT_H_
#define _SHOOT_H_

#include "CAN_receive_send.h"
#include "pid.h"
#include "User_math.h"
#include "robot_param.h"
#include "motor.h"


/* 完成角度配置后改为 1；为 0 时只发送零力矩保持帧，不执行位置运动 */
#define SHOOT_ANGLE_CONFIG_READY    1U

/* CH6 低位对应的目标角，单位：度 */
#define SHOOT_ANGLE_A_DEG           (-0.47f)

/* CH6 高位对应的目标角，单位：度 */
#define SHOOT_ANGLE_B_DEG           (-87.83f)

/* 机械结构允许的最小安全角，给机械限位预留余量，单位：度 */
#define SHOOT_SAFE_MIN_DEG          (-130.0f)

/* 机械结构允许的最大安全角，给机械限位预留余量，单位：度 */
#define SHOOT_SAFE_MAX_DEG          6.0f

/* MIT 位置环增益：提高响应速度并保留阻尼 */
#define SHOOT_MIT_KP                20.0f
#define SHOOT_MIT_KD                1.5f

/* 将配置的机械角度转换为 MIT 协议使用的弧度 */
#define SHOOT_ANGLE_A_RAD           ((SHOOT_ANGLE_A_DEG) * DEG_TO_RAD)
#define SHOOT_ANGLE_B_RAD           ((SHOOT_ANGLE_B_DEG) * DEG_TO_RAD)
#define SHOOT_SAFE_MIN_RAD          ((SHOOT_SAFE_MIN_DEG) * DEG_TO_RAD)
#define SHOOT_SAFE_MAX_RAD          ((SHOOT_SAFE_MAX_DEG) * DEG_TO_RAD)

/* 发射机构初始化和周期任务 */
void Shoot_Init(void);
void Shoot_Tasks(void);

#endif
