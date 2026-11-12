#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stdint.h"

#include "User_math.h"
#include "robot_param.h"
#include "CAN_receive_send.h"

// CAN通道数量
#define QUANTITY_OF_CAN 3 // 你所用的can的数量，（比所用的多就行了，喵板上设3别改了）

#if (USE_DJIMotor == 1)

#define QUANTITY_OF_DJIMOTOR 11 // 挂载电机最多的一路can上所用的电机
// 大疆电机参数
#define ECD_TO_ANGEL_DJI 0.043945f //(360/8192),将编码器值转化为角度制
#define DJIMOTOR_T_A 0.3           // 3508转矩常数
#define ECD_MAX 8192.0f            // 编码器最大值
#define M3508_P 19                 // M3508电机减速比（不同车的减速比不同，所以在参数配置文件中设置）
#define M2006_P 36.0f              // M2006电机减速比
#define MAX_CURRENT 16384          // M2006+M3508最大电流 20A / MAX_CURRENT
#define MAX_6020_VOL 30000         // 6020最大电压 24V / MAX_6020_VOL

/* DJImotorCAN send and receive ID */
typedef enum
{
    CAN_20063508_1_4_send_ID = 0x200,

    CAN_20063508_5_8_send_ID = 0x1FF,

    CAN_6020_1_4_send_ID = 0x1FF,

    CAN_6020_5_7_send_ID = 0x2FF,

    CAN_ID1 = 0x201,

    DJI_CAN_1 = 0,

    DJI_CAN_2 = 1,

    DJI_CAN_3 = 2,

    CAN_20063508_1_4_ID = 3,

    CAN_20063508_5_8_ID = 4,

    CAN_6020_1_4_ID = 5,

    CAN_6020_5_7_ID = 6,
} DJIcan_send_id_e;

// 6020从 CAN_1_5(对应ID1) 到 CAN_1_6020_7（对应ID7）
// 3508/2006从 CAN_1_1(对应ID1) 到 CAN_1_8（对应ID8）
// 显然，6020与3508/2006存在重叠ID,设置时请注意
typedef enum
{
    CAN_1_1 = 0,  // 0
    CAN_1_2,      // 1
    CAN_1_3,      // 2
    CAN_1_4,      // 3
    CAN_1_5,      // 4
    CAN_1_6,      // 5
    CAN_1_7,      // 6
    CAN_1_8,      // 7
    CAN_1_6020_5, // 8
    CAN_1_6020_6, // 9
    CAN_1_6020_7, // 10

    CAN_2_1, // 11
    CAN_2_2, // 12
    CAN_2_3,
    CAN_2_4,
    CAN_2_5, // 15
    CAN_2_6,
    CAN_2_7, // 17
    CAN_2_8,
    CAN_2_6020_5,
    CAN_2_6020_6,
    CAN_2_6020_7, // 21

    CAN_3_1, // 22
    CAN_3_2, // 23
    CAN_3_3,
    CAN_3_4,
    CAN_3_5,
    CAN_3_6,
    CAN_3_7,
    CAN_3_8,
    CAN_3_6020_5,
    CAN_3_6020_6,
    CAN_3_6020_7, // 32
    DJI_MOTOR_NUM,
} DJIcan_id;

typedef struct
{
    // 控制数据
    int16_t set; // 设定的电流 / 电压
    Motor_Type_e Motor_type;

    // 原始数据
    uint16_t ecd;          // 编码器数值
    int16_t speed_rpm;     // 转速RPM
    int16_t given_current; // 实际转矩电流
    uint8_t temperate;     // 温度（获取不到）
    uint16_t last_ecd;     // 上一次编码器的数值

    // 计算数据
    long long ecd_cnt;  // 编码器计数器
    double angle_cnt;   // 转过的总角度 degree
    double angle_zero;  // 编码器0点角度 degree
    double angle;       // -180~180 degree
    double round_speed; // 出轴转速 rpm
    float d_time, last_time;
} DJI_motor_data_s;

extern DJI_motor_data_s DJIMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

void DJIMotor_Init(Motor_Type_e motor_type, DJIcan_id motor_id);
void DJIMotor_SetZero(double zero_angle, DJIcan_id motor_id);
void DJIMotor_Set(int16_t val, DJIcan_id motor_id);
void DJIMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
void DJIMotor_SendCurrent(DJIcan_send_id_e CAN_Send_ID, DJIcan_send_id_e CAN_Type);
DJI_motor_data_s DJIMotor_GetData(DJIcan_id motor_id);

#endif // USE_DJIMotor

#if (USE_DMMotor == 1)

#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"

// 每个CAN总线的电机数量
#define QUANTITY_OF_DMMOTOR 6

typedef enum
{
    DM_CAN_1_1 = 0, // 0
    DM_CAN_1_2,     // 1
    DM_CAN_1_3,     // 2
    DM_CAN_1_4,     // 3
    DM_CAN_1_5,     // 4
    DM_CAN_1_6,     // 5

    DM_CAN_2_1, // 6
    DM_CAN_2_2, // 7
    DM_CAN_2_3,
    DM_CAN_2_4,
    DM_CAN_2_5, //
    DM_CAN_2_6,

    DM_CAN_3_1, //
    DM_CAN_3_2, //
    DM_CAN_3_3,
    DM_CAN_3_4,
    DM_CAN_3_5,
    DM_CAN_3_6,
    DM_MOTOR_NUM,

} DMcan_id;

// 达妙电机数据结构
typedef struct
{
    float T;
    float W;
    float Pos;

    DM_motor_t motor_data;
} DM_motor_data_s;

void DMMotor_Init(Motor_Type_e motor_type, DMcan_id motor_id);
void DMMotor_Set(DMcan_id motor_id, float pos, float vel, float tor, float kp, float kd);
void DMMotor_SetZero(double zero_angle, DMcan_id motor_id);
int DMMotor_SendCtrl(DMcan_id motor_id);
void DMMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
DM_motor_data_s DMMotor_GetData(DMcan_id motor_id);

extern DM_motor_data_s DM_Motor_data[QUANTITY_OF_CAN][6];

#endif // USE_DAMIAO_MOTOR

#if (USE_DMMotor124 == 1)

#define QUANTITY_OF_DMMOTOR124 8
#define ECD_TO_ANGEL_DM 0.043945f //(360/8192),将编码器值转化为角度制

/* DJImotorCAN send and receive ID */
typedef enum
{
    DM_J4310_1_4_send_ID = 0x3FE,

    DM_J4310_5_8_send_ID = 0x4FE,

    DM_CAN_1 = 0,

    DM_CAN_2 = 1,

    DM_CAN_3 = 2,

    DM_J4310_1_4_ID = 3,

    DM_J4310_5_8_ID = 4,

    DM_ID1 = 0x301,
} DMcan_send_id_e;

typedef enum
{
    DM_1_1 = 0, // 0
    DM_1_2,     // 1
    DM_1_3,     // 2
    DM_1_4,     // 3
    DM_1_5,     // 4
    DM_1_6,     // 5
    DM_1_7,     // 6
    DM_1_8,     // 7

    DM_2_1, // 8
    DM_2_2,
    DM_2_3,
    DM_2_4,
    DM_2_5,
    DM_2_6,
    DM_2_7,
    DM_2_8,

    DM_3_1, // 16
    DM_3_2,
    DM_3_3,
    DM_3_4,
    DM_3_5,
    DM_3_6,
    DM_3_7,
    DM_3_8,
    DM_NUM,

} DM_can_id;

typedef enum
{
    J4310 = 0,
} DMMotor_Type_e;

typedef struct
{
    // 控制数据
    int16_t set; // 设定的电流 / 电压
    DMMotor_Type_e Motor_type;

    // 原始数据
    uint16_t ecd;          // 编码器数值
    uint16_t last_ecd;     // 上一次编码器的数值
    float speed_rpm;       // 转速RPM
    int16_t given_current; // 实际转矩电流 mA
    uint8_t coil_temp;     // 线圈温度 摄氏度
    uint8_t pcb_temp;      // pcb温度

    // 计算数据
    long long ecd_cnt;  // 编码器计数器
    double angle_cnt;   // 转过的总角度 degree
    double angle_zero;  // 编码器0点角度 degree
    double angle;       // -180~180 degree
    double round_speed; // 出轴转速 rpm
} DM_motor124_data_s;

void DMMotor124_Init(DMMotor_Type_e motor_type, DM_can_id motor_id);
void DMMotor124_Set(int16_t val, DM_can_id motor_id);
void DMMotor124_GetProcessMotorData(DM_motor124_data_s *ptr, uint8_t data[]);
void DMMotor124_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
void DMMotor124_SendCurrent(DMcan_send_id_e CAN_Send_ID, DMcan_send_id_e CAN_Type);

extern DM_motor124_data_s DMMotor124_GetData(DM_can_id motor_id);

#endif // USE_DAMIAO_MOTOR

#if (USE_LZMotor == 1)
#include "LZ_motor_driver.h"

// 每个CAN总线的电机数量
#define QUANTITY_OF_LZMOTOR 6

// 电机ID枚举（支持3路CAN，每路6个电机）
typedef enum
{
    LZ_CAN_1_1 = 0,
    LZ_CAN_1_2,
    LZ_CAN_1_3,
    LZ_CAN_1_4,
    LZ_CAN_1_5,
    LZ_CAN_1_6,

    LZ_CAN_2_1,
    LZ_CAN_2_2,
    LZ_CAN_2_3,
    LZ_CAN_2_4,
    LZ_CAN_2_5,
    LZ_CAN_2_6,

    LZ_CAN_3_1,
    LZ_CAN_3_2,
    LZ_CAN_3_3,
    LZ_CAN_3_4,
    LZ_CAN_3_5,
    LZ_CAN_3_6,

    LZ_MOTOR_NUM
} LZ_Motor_ID_t;

void LZMotor_init(LZ_Motor_ID_t motor_id);
void LZMotor_set_params(LZ_Motor_ID_t motor_id, float pos, float vel, float tor, float kp, float kd);
int LZMotor_send_command(LZ_Motor_ID_t motor_id);
LZ_Motor_t LZMotor_get(LZ_Motor_ID_t motor_id);
void LZMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

// 获取电机对象指针
extern LZ_Motor_t LZ_Motors[QUANTITY_OF_CAN][QUANTITY_OF_LZMOTOR];

#endif // USE_LINGZU_MOTOR

#if (USE_UNTREE_MOTOR == 1)

#include "usart.h"
typedef struct
{
    // 待发送的各项数据
    uint8_t motor_id; // 电机ID，0xBB代表全部电机
    uint8_t mode;     // 0:空闲, 5:开环转动, 10:闭环FOC控制
    // 实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
    float T;   // 期望关节的输出力矩（减速器实际的力矩）（Nm）,×256 倍描述
    float W;   // 期望关节速度（电机本身的速度）(rad/s),×128 倍描述
    float Pos; // 期望关节位置（rad）×16384/2π 倍描述
    float K_P; // 关节刚度系数,×2048 倍描述
    float K_W; // 关节速度系数,×1024 倍描述
    uint8_t Tx_buff[34];

} UntreeA1_MOTOR_send_s;

typedef struct
{
    // 定义 接收数据
    uint8_t correct; // 接收数据是否完整（1完整，0不完整）
    // 解读得出的电机数据
    uint8_t motor_id; // 电机ID
    uint8_t mode;     // 0:空闲, 5:开环转动, 10:闭环FOC控制
    int32_t Temp;     // 温度
    uint8_t MError;   // 错误码
    float T;          // 当前实际减速器后输出力矩,×256 倍描述
    float W;          // 当前实际电机速度（高速）,×128 倍描述
    float LW;         // 当前实际电机速度（低速也表示当前电机实际转速，但是已经过滤波，所以会有延迟，不推荐使用）
    int32_t Acc;      // 电机转子加速度
    float Pos;        // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）×16384/2π 倍描述

    float gyro[3]; // 电机驱动板6轴传感器数据
    float acc[3];
} UntreeA1_MOTOR_receive_s;

typedef struct
{
    uint8_t motor_id; // 电机ID
    uint8_t mode;     // 0:空闲, 5:开环转动, 10:闭环FOC控制
    struct
    {
        float T;   // 设置力矩（N·M)
        float W;   // 设置角速度(rad/s)(内转子)
        float Pos; // 输出轴角度
        float K_P; // 关节刚度系数
        float K_W; // 关节速度系数
    } Set;
    struct
    {
        float T;          // 力矩（N·M)
        float W;          // 角速度(rad/s)(输出轴)
        float LW;         // 低通滤波后的角速度，有延迟。
        float Pos;        // 输出轴角度(rad)
        float Pos_insert; // 内转子角度
        float Acc;        // 电机转动加速度
        int32_t Temp;     // 温度
        uint8_t MError;   // 错误码
        float gyro[3];    // 驱动板陀螺仪角速度
        float acc[3];     // 驱动板陀螺仪加速度
    } Now;
    float zero_pos;
    UntreeA1_MOTOR_send_s Send_data;
    UntreeA1_MOTOR_receive_s Receive_data;
} UntreeA1_Motor_data_s;

extern UntreeA1_Motor_data_s UntreeA1_Motor_data[6];

typedef enum
{
    Uart2_0 = 0,
    Uart2_1 = 1,
    Uart2_2 = 2,
    Uart3_0 = 3,
    Uart3_1 = 4,
    Uart3_2 = 5,
} Untree_id;

void UntreeA1_init(uint8_t Motor_id);
uint8_t UntreeA1_Resovle_data(uint8_t receive_buf[], UART_HandleTypeDef *huart);
void UntreeA1_Send_data(uint8_t id);
void UntreeA1_set(uint8_t id, float T, float W, float K_W, float pos, float K_P);

#endif // USE_UNTREE_MOTOR

#if (USE_LINGKONG_MOTOR == 1)
// 这么屎的电机以后还是别用了，一路can就准备两个临时用用吧
typedef enum
{
    LK_CAN_1_1 = 0,
    LK_CAN_1_2,

    LK_CAN_2_1,
    LK_CAN_2_2,

    LK_CAN_3_1,
    LK_CAN_3_2,
} LK_Can_id_e;

typedef struct
{
    uint8_t id;         // 电机ID
    float W;            // 转速(rad/s)
    float T_set;        // 设置转矩N*M
    float T_now;        // 实际转矩N*M
    float pos;          // 电机角度（rad,-PI~PI）
    float last_pos;     // 上一次电机的角度
    double Angle_cnt;   // 转换后多圈角度（rad)
    int8_t temperature; // 温度

} LK_motor_data_s;

extern LK_motor_data_s LK_motor_data[3][2];

void LK_Motor_init(LK_Can_id_e motor_id);
void LK_Motor_set(LK_Can_id_e motor_id, float T);
void LK_Motor_send_data(LK_Can_id_e motor_id);
void LK_ask_for_angle_cnt(LK_Can_id_e motor_id);
void LK_Motor_decode_data(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

#endif // DEBUG

#endif // !__MOTOR_H__