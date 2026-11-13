#include "Shoot.h"
#include "Global_status.h"
#include "User_math.h"

/* 检查角度配置是否完整，并且 A/B 目标角位于安全范围内 */
static uint8_t Shoot_ConfigValid(void)
{
    return SHOOT_ANGLE_CONFIG_READY == 1U &&
           SHOOT_SAFE_MIN_RAD < SHOOT_SAFE_MAX_RAD &&
           SHOOT_ANGLE_A_RAD >= SHOOT_SAFE_MIN_RAD &&
           SHOOT_ANGLE_A_RAD <= SHOOT_SAFE_MAX_RAD &&
           SHOOT_ANGLE_B_RAD >= SHOOT_SAFE_MIN_RAD &&
           SHOOT_ANGLE_B_RAD <= SHOOT_SAFE_MAX_RAD;
}

void Shoot_Init(void)
{
    /* 初始化 CAN1 上的两台灵足电机 */
    LZMotor_init(LZ_CAN_1_1);
    LZMotor_init(LZ_CAN_1_2);

    /* 初始化 CAN3 上的达妙 8009P 发射电机 */
    DMMotor_Init(DM_4310, DM_CAN_3_1);
}

void Shoot_Tasks(void)
{
    /* 获取达妙电机当前的反馈、电流和增益数据 */
    DM_motor_data_s motor = DMMotor_GetData(DM_CAN_3_1);
    float target_rad;

    /*
     * MIT 模式状态 1 表示电机已经使能。
     * 冷启动时电机可能比 MCU 晚上电，如果第一次使能帧丢失，
     * 这里持续重发使能帧，避免必须重新烧录或手动使能。
     */
    if (motor.motor_data.para.state != 1) {
        DMMotor_SendCtrl(DM_CAN_3_1);
        return;
    }

    /* 配置未完成或遥控器锁定时，使用零力矩帧释放 MIT 位置环 */
    if (!Shoot_ConfigValid() || Global.Control.mode != RC) {
        DMMotor_Set(DM_CAN_3_1, motor.motor_data.para.pos,
                    0.0f, 0.0f, 0.0f, 0.0f);
        DMMotor_SendCtrl(DM_CAN_3_1);
        return;
    }

    target_rad = fp32_constrain(Global.Shoot.shoot_angle.pitch_angle,
                                SHOOT_SAFE_MIN_RAD, SHOOT_SAFE_MAX_RAD);

    /* 使用 MIT 位置、速度、Kp、Kd 和前馈力矩控制电机 */
    DMMotor_Set(DM_CAN_3_1, target_rad,0.0f, 0.0f,20.0f,1.5f);

    /* 计算并发送 CAN3 上的 MIT 控制帧 */
    DMMotor_SendCtrl(DM_CAN_3_1);
}
