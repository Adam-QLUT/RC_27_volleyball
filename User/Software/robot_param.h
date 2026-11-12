/**
 * @file  robot_param.h
 * @brief 全向轮步兵的兵种、双板通信及硬件参数配置。
 *
 * @note Gimbal 与 Chassis 工程各保留一份本文件。
 *       除 BOARD_GIMBAL / BOARD_CHASSIS 外，两块板的配置必须保持一致。
 */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

/*================================================ 全局功能 ===========================================================*/

/* 模式开关：0=关闭，1=启用。 */
#define CONTROL_TYPE (1)  /* 正常控制模式 */
#define DEBUG_TYPE   (0)  /* 调试模式 */


/* 电机驱动：0=不编译，1=编译。 */
#define USE_DJIMotor   (1)  /* DJI M2006 / M3508 / GM6020 */
#define USE_DMMotor    (1)  /* 达妙 MIT 电机 */
#define USE_DMMotor124 (0)  /* 达妙 1 拖 4 模式 */
#define USE_LZMotor    (1)  /* 灵足电机 */

/*================================================ 电机类型 ===========================================================*/

typedef enum
{
    DJI_M2006 = 0,
    DJI_M3508,
    DJI_GM6020,
    DM_4310,
    DM_4340,
    LZ_00,
} Motor_Type_e;

/*
 * 达妙反馈 MASTER_ID 基址。
 * 原步兵电机配置为 0x01~0x06；不要直接改成 fold 使用的 0x11~0x16。
 */
#ifndef DM_FEEDBACK_ID_BASE
#define DM_FEEDBACK_ID_BASE 0x01U
#endif

/*================================================ 全向轮底盘 =========================================================*/

/* 底盘几何参数。 */
#define R_body     (0.48524f)  /* m，底盘中心到轮组安装中心的距离 */
#define r_wheel    (0.064f)  /* m，轮子半径 */
#define WHEEL_RATIO (19.2032f)   /* 底盘 M3508 减速比 */

/* 驱动电机 CAN ID；顺序为左前、右前、左后、右后。 */
#define WHEEL_FL CAN_2_3
#define WHEEL_FR CAN_2_4
#define WHEEL_BL CAN_2_2
#define WHEEL_BR CAN_2_1

/*================================================ 云台参数 ===========================================================*/

#define YAWMotor   DM_CAN_3_2
#define PITCHMotor DM_CAN_1_1

/* 电机型号。 */
#define GIMBAL_YAW_MOTOR_TYPE   ((Motor_Type_e)DM_4310)
#define GIMBAL_PITCH_MOTOR_TYPE ((Motor_Type_e)DM_4310)



#ifdef __cplusplus
}
#endif

#endif 
/* INCLUDED_ROBOT_PARAM_H */
