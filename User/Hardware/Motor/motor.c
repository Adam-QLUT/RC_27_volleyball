#include "motor.h"
#include "math.h"
#include "string.h"
#include "Stm32_time.h"

#define abs(a) a > 0 ? a : -a

#if (USE_DJIMotor == 1)
// 电机数据定义
DJI_motor_data_s DJIMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

/**
 * @brief 大疆电机初始化，暂时只用来初始化类型
 *
 * @param motor_type 电机种类
 * @param motor_id 电机can通道及ID
 */
void DJIMotor_Init(Motor_Type_e motor_type, DJIcan_id motor_id)
{
    uint8_t cantype = motor_id / 11; // 获得电机所在can路
    uint8_t canid = motor_id % 11;   // 得到电机ID值；

    DJIMotor_data[cantype][canid].Motor_type = motor_type; // 初始化相应电机
}

/**
 * @brief 设置大疆电机零点
 *
 * @param zero_angle 零点角度
 * @param motor_id 电机ID
 */
void DJIMotor_SetZero(double zero_angle, DJIcan_id motor_id)
{
    uint8_t cantype = motor_id / 11; // 获得电机所在can路
    uint8_t canid = motor_id % 11;   // 得到电机ID值；

    DJIMotor_data[cantype][canid].angle_zero = zero_angle; // 初始化相应电机
}

/**
 * @brief 设置大疆电机电流
 *
 * @param val 电流值
 * @param motor_id 电机can通道及ID
 */
void DJIMotor_Set(int16_t val, DJIcan_id motor_id)
{
    DJIMotor_data[motor_id / 11][motor_id % 11].set = val; // 设置电流
}

/**
 * @brief 获取大疆电机数据
 *
 * @param motor_id 电机can通道及ID
 * @return DJI_motor_data_s 电机数据结构体。
 */
DJI_motor_data_s DJIMotor_GetData(DJIcan_id motor_id) // 获取马达数据
{
    return DJIMotor_data[motor_id / 11][motor_id % 11];
}

/**
 * @brief 大疆电机CAN数据接受以及处理
 *
 * @param ptr 电机数据
 * @param data can数据
 */
void DJIMotor_GetProcessMotorData(DJI_motor_data_s *ptr, uint8_t data[])
{
    // get raw data
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->temperate = (data)[6];

    // process the data
    // count cnt
    if ((ptr)->last_ecd > 7000 && (ptr)->ecd < 1000)
        (ptr)->ecd_cnt += ((ECD_MAX - (ptr)->last_ecd) + (ptr)->ecd);
    else if ((ptr)->last_ecd < 1000 && (ptr)->ecd > 7000)
        (ptr)->ecd_cnt -= ((ECD_MAX - (ptr)->ecd) + (ptr)->last_ecd);
    else
        (ptr)->ecd_cnt += ((ptr)->ecd - (ptr)->last_ecd);
    // process data
    (ptr)->angle_cnt = (ptr)->ecd_cnt * ECD_TO_ANGEL_DJI;
    // 根据不同电机进行不同处理
    if ((ptr)->Motor_type == DJI_GM6020)
    {
        // 计算出轴转速
        (ptr)->round_speed = (ptr)->speed_rpm;

        // 计算相对角度 -180~180 谨防精度丢失 总角度过大时
        float angle = (ptr)->angle_cnt - (ptr)->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // 处于-180度
                angle -= (mul + 1) * 180;
            else // 处于180度
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // 处于180度
                angle += (mul + 1) * 180;
            else // 处于-180度
                angle += mul * 180;
        }
        (ptr)->angle = angle;
    }
    else if ((ptr)->Motor_type == DJI_M3508)
    {
        (ptr)->round_speed = (ptr)->speed_rpm / M3508_P;
    }
    else if ((ptr)->Motor_type == DJI_M2006)
    {
        (ptr)->round_speed = (ptr)->speed_rpm / M2006_P;
    }
    (ptr)->d_time = Get_SysTime_ms() - (ptr)->last_time;
    (ptr)->last_time = Get_SysTime_ms();
}

/**
 * @brief 大疆电机can数据处理
 *
 * @param hfdcan CAN通道
 * @param id can标识符
 * @param data can数据
 */
void DJIMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id < CAN_ID1)
        return;
    if (id - CAN_ID1 <= 11) // 防止数组溢出
    {
        if (hfdcan == &hfdcan1)
        {
            DJIMotor_GetProcessMotorData(&DJIMotor_data[0][id - CAN_ID1], data);
        }
        else if (hfdcan == &hfdcan2)
        {
            DJIMotor_GetProcessMotorData(&DJIMotor_data[1][id - CAN_ID1], data);
        }
        else if (hfdcan == &hfdcan3)
        {
            DJIMotor_GetProcessMotorData(&DJIMotor_data[2][id - CAN_ID1], data);
        }
    }
}

/**
 * @brief 大疆电机电流值发送，建议放freertos里定期发送
 *
 */
void DJIMotor_SendCurrent(DJIcan_send_id_e CAN_Send_ID, DJIcan_send_id_e CAN_Type)
{
    uint8_t can_send_data[8];
    uint8_t canid = 0;
    uint16_t identifier = 0;
    FDCAN_HandleTypeDef *hcan = NULL;

    if (hfdcan1.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan1);
    if (hfdcan2.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan2);
    if (hfdcan3.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan3);

    // 判断电机类型以及电机ID
    switch (CAN_Send_ID)
    {
    case CAN_20063508_1_4_ID:
    {
        identifier = CAN_20063508_1_4_send_ID;
        canid = 0;
        break;
    }
    case CAN_20063508_5_8_ID:
    {
        identifier = CAN_20063508_5_8_send_ID;
        canid = 4;
        break;
    }
    case CAN_6020_1_4_ID:
    {
        identifier = CAN_6020_1_4_send_ID;
        canid = 4;
        break;
    }
    case CAN_6020_5_7_ID:
    {
        identifier = CAN_6020_5_7_send_ID;
        canid = 8;
        break;
    }
    default:
        return;
    }

    // 判断CAN路
    switch (CAN_Type)
    {
    case DJI_CAN_1:
        hcan = &hfdcan1;
        break;
    case DJI_CAN_2:
        hcan = &hfdcan2;
        break;
    case DJI_CAN_3:
        hcan = &hfdcan3;
        break;
    default:
        return;
    }

    // 三路can依次赋值
    for (int i = 0; i < 8; i += 2)
    {
        can_send_data[i] = (DJIMotor_data[CAN_Type][canid].set >> 8);
        can_send_data[i + 1] = DJIMotor_data[CAN_Type][canid].set;
        canid++;
    }

    // 发送CAN数据
    Fdcanx_SendData(hcan, identifier, can_send_data, 8);
}

#endif // USE_DJIMotor

#if (USE_DMMotor == 1)

// 达妙电机数据定义
DM_motor_data_s DM_Motor_data[QUANTITY_OF_CAN][QUANTITY_OF_DMMOTOR];

/**
 * @brief 使能达妙电机
 * @param motor_id 电机ID
 */
void DMMotor_Enable(DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    switch (cantype)
    {
    case 0:
        dm_motor_enable(&hfdcan1, &DM_Motor_data[cantype][canid].motor_data);
        break;
    case 1:
        dm_motor_enable(&hfdcan2, &DM_Motor_data[cantype][canid].motor_data);
        break;
    case 2:
        dm_motor_enable(&hfdcan3, &DM_Motor_data[cantype][canid].motor_data);
        break;
    default:
        break;
    }
}

/**
 * @brief 失能达妙电机
 * @param motor_id 电机ID
 */
void DMMotor_Disable(uint8_t motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;
    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；
    dm_motor_disable(&hfdcan1, &DM_Motor_data[cantype][canid].motor_data);
}

/**
 * @brief 设置达妙电机控制模式
 * @param motor_id 电机ID
 * @param mode 控制模式
 */
void DMMotor_SetMode(DMcan_id motor_id, DM_mode_e mode)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    DM_Motor_data[cantype][canid].motor_data.ctrl.mode = mode;
}

/**
 * @brief 达妙电机初始化
 * @param motor_id 电机ID (0-5)
 */
void DMMotor_Init(Motor_Type_e motor_type, DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    memset(&DM_Motor_data[cantype][canid], 0, sizeof(DM_motor_data_s));

    DM_Motor_data[cantype][canid].motor_data.id = canid + 1;
    DM_Motor_data[cantype][canid].motor_data.mst_id = canid + 1;
    DM_Motor_data[cantype][canid].motor_data.tmp.read_flag = 1;
    DM_Motor_data[cantype][canid].motor_data.tmp.PMAX = 12.5f;
    DM_Motor_data[cantype][canid].motor_data.tmp.VMAX = 30.0f;
    DM_Motor_data[cantype][canid].motor_data.tmp.TMAX = 10.0f;
    DM_Motor_data[cantype][canid].motor_data.ctrl.mode = mit_mode; // 默认MIT模式

    // 使能电机
    DMMotor_Enable(motor_id);
}

/**
 * @brief 设置达妙电机控制参数
 * @param motor_id 电机ID
 * @param pos 位置设定值 (rad)
 * @param vel 速度设定值 (rad/s)
 * @param tor 扭矩设定值 (N*M)
 * @param kp 位置比例增益(N/r)
 * @param kd 位置微分增益(N*s/r)
 */
void DMMotor_Set(DMcan_id motor_id, float pos, float vel, float tor, float kp, float kd)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    DM_motor_t *m = &DM_Motor_data[cantype][canid].motor_data;
    m->ctrl.pos_set = pos;
    m->ctrl.vel_set = vel;
    m->ctrl.tor_set = tor;
    m->ctrl.kp_set = kp;
    m->ctrl.kd_set = kd;
}

/**
 * @brief 设置达妙电机零点
 * @param zero_angle 零点角度
 * @param motor_id 电机ID
 */
void DMMotor_SetZero(double zero_angle, DMcan_id motor_id)
{
    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    DM_Motor_data[cantype][canid].motor_data.angle_zero = zero_angle; // 初始化相应电机
}

/**
 * @brief 达妙电机CAN数据处理
 * @param hfdcan CAN通道
 * @param id can标识符
 * @param data can数据
 */
void DMMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id == 0U || id > 0x06U)
        return;
    uint8_t cantype; // 获得电机所在can路
    uint8_t canid;   // 得到电机ID值；
    id--;
    cantype = 0;
    if (hfdcan == &hfdcan1)
    { // 达妙电机反馈ID
        cantype = 0;
    }
    else if (hfdcan == &hfdcan2)
    {
        cantype = 1;
    }
    else if (hfdcan == &hfdcan3)
    {
        cantype = 2;
    }
    else
    {
        return;
    }
    canid = id;
    // 更新所有达妙电机的反馈数据
    dm_motor_fbdata(&DM_Motor_data[cantype][canid].motor_data, data);
}

/**
 * @brief 达妙电机控制命令发送
 */
int DMMotor_SendCtrl(DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
         return 0;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；
    FDCAN_HandleTypeDef *hcan = Get_CanHandle(cantype);

    if (DM_Motor_data[cantype][canid].motor_data.para.state == 1)
    { // 如果电机已使能
        dm_motor_ctrl_send(hcan, &DM_Motor_data[cantype][canid].motor_data);
        return 1;
    }
    else if (DM_Motor_data[cantype][canid].motor_data.para.state == 0)
    {
        DMMotor_Enable(motor_id);
        return 0;
    }
    else 
    {
        dm_motor_clear_err(hcan,&DM_Motor_data[cantype][canid].motor_data);
        DMMotor_Disable(motor_id);
        return 0;
    }
}

/**
 * @brief 获取达妙电机数据
 *
 * @param motor_id 电机can通道及ID
 * @return DM_motor_data_s 电机数据结构体。
 */
DM_motor_data_s DMMotor_GetData(DMcan_id motor_id) // 获取马达数据
{
    return DM_Motor_data[motor_id / 6][motor_id % 6];
}

#endif // USE_DMMotor

#if (USE_DMMotor124 == 1)

// 电机数据定义
DM_motor124_data_s DMMotor124_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

/**
 * @brief达妙电机初始化
 *
 * @param motor_type 电机种类：J4310
 * @param motor_id 电机can通道及ID
 */
void DMMotor124_Init(DMMotor_Type_e motor_type, DM_can_id motor_id)
{
    uint8_t cantype = (motor_id + 1) / 8; // 获得电机所在can路
    uint8_t canid = motor_id % 8;         // 得到电机ID值；

    DMMotor124_data[cantype][canid].Motor_type = motor_type; // 初始化相应电机
}

/**
 * @brief 设置DAMIAO电机电流
 *
 * @param val 电流值
 * @param motor_id 电机can通道及ID
 */
void DMMotor124_Set(int16_t val, DM_can_id motor_id)
{
    DMMotor124_data[(motor_id + 1) / 8][motor_id % 8].set = val; // 设置电流
}

/**
 * @brief 获取DAMIAO电机数据
 *
 * @param motor_id 电机can通道及ID
 * @return DJI_motor_data_s 电机数据结构体。
 */
DM_motor124_data_s DMMotor124_GetData(DM_can_id motor_id) // 获取马达数据
{
    return DMMotor124_data[(motor_id + 1) / 8][motor_id % 8];
}

/**
 * @brief DAMIAO电机CAN数据接受以及处理
 *
 * @param ptr 电机数据
 * @param data can数据
 */
void DMMotor124_GetProcessMotorData(DM_motor124_data_s *ptr, uint8_t data[])
{
    // get raw data
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    (ptr)->speed_rpm = 0.01f * (int16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->coil_temp = (data)[6];
    (ptr)->pcb_temp = (data)[7];

    // process the data
    // count cnt
    if ((ptr)->last_ecd > 7000 && (ptr)->ecd < 1000)
        (ptr)->ecd_cnt += ((ECD_MAX - (ptr)->last_ecd) + (ptr)->ecd);
    else if ((ptr)->last_ecd < 1000 && (ptr)->ecd > 7000)
        (ptr)->ecd_cnt -= ((ECD_MAX - (ptr)->ecd) + (ptr)->last_ecd);
    else
        (ptr)->ecd_cnt += ((ptr)->ecd - (ptr)->last_ecd);
    // process data
    (ptr)->angle_cnt = (ptr)->ecd_cnt * ECD_TO_ANGEL_DM;
    // 根据不同电机进行不同处理
    if ((ptr)->Motor_type == J4310)
    {
        // 计算出轴转速
        (ptr)->round_speed = (ptr)->speed_rpm;

        // 计算相对角度 -180~180 谨防精度丢失 总角度过大时
        float angle = (ptr)->angle_cnt - (ptr)->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // 处于-180度
                angle -= (mul + 1) * 180;
            else // 处于180度
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // 处于180度
                angle += (mul + 1) * 180;
            else // 处于-180度
                angle += mul * 180;
        }
        (ptr)->angle = angle;
    }
    else
    {
    }
}

/**
 * @brief DAMIAO电机can数据处理
 *
 * @param hfdcan CAN通道
 * @param id can标识符
 * @param data can数据
 */
void DMMotor124_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id - DM_ID1 <= 8 && id - DM_ID1 >= 0) // 防止数组溢出
    {
        if (hfdcan == &hfdcan1)
        {
            DMMotor124_GetProcessMotorData(&DMMotor124_data[0][id - DM_ID1], data);
        }
        else if (hfdcan == &hfdcan2)
        {
            DMMotor124_GetProcessMotorData(&DMMotor124_data[1][id - DM_ID1], data);
        }
        else if (hfdcan == &hfdcan3)
        {
            DMMotor124_GetProcessMotorData(&DMMotor124_data[2][id - DM_ID1], data);
        }
    }
}

/**
 * @brief DAMIAO电机电流值发送，建议放freertos里定期发送
 *
 */
void DMMotor124_SendCurrent(DMcan_send_id_e CAN_Send_ID, DMcan_send_id_e CAN_Type)
{
    uint8_t can_send_data[8];
    uint8_t canid;
    uint16_t identifier;
    FDCAN_HandleTypeDef *hcan;

    if (hfdcan1.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan1);
    if (hfdcan2.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan2);
    if (hfdcan3.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan3);

    // 判断电机类型以及电机ID
    switch (CAN_Send_ID)
    {
    case DM_J4310_1_4_ID:
    {
        identifier = DM_J4310_1_4_send_ID;
        canid = 0;
        break;
    }

    case DM_J4310_5_8_ID:
    {
        identifier = DM_J4310_5_8_send_ID;
        canid = 4;
        break;
    }
    }

    // 判断CAN路
    switch (CAN_Type)
    {
    case DJI_CAN_1:
        hcan = &hfdcan1;
        break;
    case DJI_CAN_2:
        hcan = &hfdcan2;
        break;
    case DJI_CAN_3:
        hcan = &hfdcan3;
        break;
    }

    // 三路can依次赋值
    for (int i = 0; i < 8; i += 2)
    {
        can_send_data[i] = (DMMotor124_data[CAN_Type][canid].set);          // 低 8 位
        can_send_data[i + 1] = (DMMotor124_data[CAN_Type][canid].set >> 8); // 高 8 位
        canid++;
    }

    // 发送CAN数据
    Fdcanx_SendData(hcan, identifier, can_send_data, 8);
}

#endif // USE_DMMotor

#if (USE_LZMotor == 1)
// 电机数据数组
LZ_Motor_t LZ_Motors[QUANTITY_OF_CAN][QUANTITY_OF_LZMOTOR];

/**
 * @brief 使能灵足电机
 */
void LZMotor_enable(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；
   // lz_clear_fault(cantype, LZ_Motors[cantype][canid].id);
    lz_enable_motor(cantype, LZ_Motors[cantype][canid].id);
    LZ_Motors[cantype][canid].mode = LZ_MODE_MIT; // 默认使能后进入MIT模式
}

/**
 * @brief 失能灵足电机
 */
void LZMotor_disable(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    lz_disable_motor(cantype, LZ_Motors[cantype][canid].id);
    LZ_Motors[cantype][canid].mode = LZ_MODE_DISABLE;
}

/**
 * @brief 初始化灵足电机
 */
void LZMotor_init(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    // 只重置控制目标，不覆盖已收到的反馈状态，避免上电顺序导致反馈被清空。
    LZ_Motors[cantype][canid].id = canid + 1;
    LZ_Motors[cantype][canid].master_id = canid + 1;
    LZ_Motors[cantype][canid].mode = LZ_MODE_MIT;
    LZ_Motors[cantype][canid].pos_set = 0.0f;
    LZ_Motors[cantype][canid].vel_set = 0.0f;
    LZ_Motors[cantype][canid].tor_set = 0.0f;
    LZ_Motors[cantype][canid].kp_set = 0.0f;
    LZ_Motors[cantype][canid].kd_set = 0.0f;
    LZ_Motors[cantype][canid].current_limit = 0.0f;

    // 使能电机
    LZMotor_enable(motor_id);
}

/**
 * @brief 设置灵足电机控制参数
 */
void LZMotor_set_params(LZ_Motor_ID_t motor_id, float pos, float vel, float tor, float kp, float kd)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    LZ_Motors[cantype][canid].pos_set = pos;
    LZ_Motors[cantype][canid].vel_set = vel;
    LZ_Motors[cantype][canid].tor_set = tor;
    LZ_Motors[cantype][canid].kp_set = kp;
    LZ_Motors[cantype][canid].kd_set = kd;
}

/**
 * @brief 发送灵足电机控制命令
 */
int LZMotor_send_command(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return 0;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    LZ_Motor_t *motor = &LZ_Motors[cantype][canid];
    if (motor->state.status == 2 && motor->state.fault == 0)
    {
        lz_send_mit_params(cantype, motor->id, motor->pos_set, motor->vel_set,
                           motor->kp_set, motor->kd_set, motor->tor_set);
        return 1;
    }
    else if (motor->state.status == 0 && motor->state.fault == 0)
    {
        LZMotor_enable(motor_id);
        return 0;
    }
    else
    {
        LZMotor_disable(motor_id);
        lz_clear_fault(cantype, motor->id);
        return 0;
    }
        
}

/**
 * @brief 获取电机对象指针
 */
LZ_Motor_t LZMotor_get(LZ_Motor_ID_t motor_id)
{

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    return LZ_Motors[cantype][canid];
}

/**
 * @brief 灵足电机CAN数据处理
 */
void LZMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id > 10)
        return;
    uint8_t motor_id = (data[0] - 1);

    // 确定CAN总线
    uint8_t can_bus = 0;
    if (hfdcan == &hfdcan2)
        can_bus = 1;
    else if (hfdcan == &hfdcan3)
        can_bus = 2;

    LZ_Motors[can_bus][motor_id].state.angle_last = LZ_Motors[can_bus][motor_id].state.angle;
    LZ_Motors[can_bus][motor_id].state.angle = uint_to_float((data[1] << 8) | (data[2]), P_MIN, P_MAX, 16);
    LZ_Motors[can_bus][motor_id].state.velocity = uint_to_float((data[3] << 4) | (data[4] >> 4), V_MIN, V_MAX, 12);
    LZ_Motors[can_bus][motor_id].state.torque = uint_to_float((data[4] << 8) | (data[5]), T_MIN, T_MAX, 12);
    LZ_Motors[can_bus][motor_id].state.status = (data[6] >> 6) & 0x03;
    LZ_Motors[can_bus][motor_id].state.fault = (data[6] >> 5) & 0x01;
    LZ_Motors[can_bus][motor_id].state.temperature = (((data[6] & 0x0F) << 8) | data[7]) * 0.1;

    // 数据处理
    //  count cnt
    if (LZ_Motors[can_bus][motor_id].state.sign == 0)
    {
        LZ_Motors[can_bus][motor_id].state.sign++;
        LZ_Motors[can_bus][motor_id].state.angle_last = LZ_Motors[can_bus][motor_id].state.angle;
    }
    if (LZ_Motors[can_bus][motor_id].state.angle_last > 12 && LZ_Motors[can_bus][motor_id].state.angle < -12)
        LZ_Motors[can_bus][motor_id].state.angle_cnt += ((P_MAX - LZ_Motors[can_bus][motor_id].state.angle_last) + (LZ_Motors[can_bus][motor_id].state.angle + P_MAX));
    else if (LZ_Motors[can_bus][motor_id].state.angle_last < -12 && LZ_Motors[can_bus][motor_id].state.angle > 12)
        LZ_Motors[can_bus][motor_id].state.angle_cnt -= ((P_MAX - LZ_Motors[can_bus][motor_id].state.angle) + LZ_Motors[can_bus][motor_id].state.angle_last + P_MAX);
    else
        LZ_Motors[can_bus][motor_id].state.angle_cnt += (LZ_Motors[can_bus][motor_id].state.angle - LZ_Motors[can_bus][motor_id].state.angle_last);
    LZ_Motors[can_bus][motor_id].d_time = Get_SysTime_ms() - LZ_Motors[can_bus][motor_id].last_time;
    LZ_Motors[can_bus][motor_id].last_time = Get_SysTime_ms();
}

#endif // USE_LZMotor

#if (USE_UNTREE_MOTOR == 1)

UntreeA1_Motor_data_s UntreeA1_Motor_data[6];

static uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

void UntreeA1_init(uint8_t Motor_id)
{
    memset(&UntreeA1_Motor_data[Motor_id], 0, sizeof(UntreeA1_Motor_data_s));
    UntreeA1_Motor_data[Motor_id].motor_id = Motor_id % 3;
    UntreeA1_Motor_data[Motor_id].mode = 10; // 默认FOC闭环模式
}

/**
 * @brief 宇树A1电机数据解析
 *
 * @param receive_buf 数据缓冲区
 * @param huart 接受到的数据句柄
 * @return uint8_t 1 数据正常，0 数据异常
 */
uint8_t UntreeA1_Resovle_data(uint8_t receive_buf[], UART_HandleTypeDef *huart)
{
    // 数据解析
    int16_t receive_T = (int16_t)(((uint16_t)receive_buf[13] << 8) + (uint16_t)receive_buf[12]);
    int16_t receive_W = (int16_t)(((uint16_t)receive_buf[15] << 8) + (uint16_t)receive_buf[14]);
    int32_t receive_LW = (int32_t)(((uint32_t)receive_buf[19] << 24) + ((uint32_t)receive_buf[18] << 16) + ((uint32_t)receive_buf[17] << 8) + receive_buf[16]);
    int16_t receive_Acc = (int16_t)(((uint16_t)receive_buf[27] << 8) + (uint16_t)receive_buf[26]);
    int32_t receive_Pos = (int32_t)(((uint32_t)receive_buf[33] << 24) + ((uint32_t)receive_buf[32] << 16) + ((uint32_t)receive_buf[31] << 8) + receive_buf[30]);

    unsigned long receive_crc = ((uint32_t)receive_buf[77] << 24) + ((uint32_t)receive_buf[76] << 16) + ((uint32_t)receive_buf[75] << 8) + (uint32_t)receive_buf[74]; // 接收到的得的到CRC
    unsigned long calculate_crc = crc32_core((uint32_t *)receive_buf, 18);                                                                                            // 计算得的到CRC

    uint8_t motor_id = receive_buf[2];
    uint8_t mode = receive_buf[4];
    uint8_t Temp = receive_buf[6];
    uint8_t MError = receive_buf[7];
    // 检验数据是否正确
    if (calculate_crc != receive_crc)
    {
        return 0;
    }
    // 数据正确，开始赋值
    uint8_t id = 0;
    if (huart == &huart2)
        id = motor_id;
    else if (huart == &huart3)
        id = motor_id + 3;
    UntreeA1_Motor_data[id].Receive_data.Acc = receive_Acc;
    UntreeA1_Motor_data[id].Receive_data.mode = mode;
    UntreeA1_Motor_data[id].Receive_data.correct = 1;
    UntreeA1_Motor_data[id].Receive_data.MError = MError;
    UntreeA1_Motor_data[id].Receive_data.motor_id = id;
    UntreeA1_Motor_data[id].Receive_data.Pos = receive_Pos;
    UntreeA1_Motor_data[id].Receive_data.LW = receive_LW;
    UntreeA1_Motor_data[id].Receive_data.T = receive_T;
    UntreeA1_Motor_data[id].Receive_data.Temp = Temp;

    // 数据计算
    UntreeA1_Motor_data[id].Now.T = ((float)receive_T) / 256 * 9.1f;
    UntreeA1_Motor_data[id].Now.W = ((float)receive_W) / 128 / 9.1f;
    UntreeA1_Motor_data[id].Now.LW = ((float)receive_LW) / 16384 * 2 * PI;
    UntreeA1_Motor_data[id].Now.Acc = (int)receive_Acc / 9.1f;
    UntreeA1_Motor_data[id].Now.Pos = (2 * PI * ((float)receive_Pos) / 16384 / 9.1f);
    UntreeA1_Motor_data[id].Now.MError = MError;
    UntreeA1_Motor_data[id].Now.Temp = Temp;

    return 1;
}

/**
 * @brief 宇树A1发送信息
 *
 * @param id 电机ID
 */
void UntreeA1_Send_data(uint8_t id)
{
    int16_t Send_T = UntreeA1_Motor_data[id].Set.T * 256;
    int16_t Send_W = UntreeA1_Motor_data[id].Set.W * 128;
    int32_t Send_Pos = (int32_t)((UntreeA1_Motor_data[id].Set.Pos / 2 * PI) * 16384.0f * 9.1f);
    int16_t Send_KP = UntreeA1_Motor_data[id].Set.K_P * 2048;
    int16_t Send_KW = UntreeA1_Motor_data[id].Set.K_W * 1024;

    UntreeA1_Motor_data[id].Send_data.Tx_buff[0] = 0xFE;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[1] = 0xEE;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[2] = UntreeA1_Motor_data[id].motor_id;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[4] = UntreeA1_Motor_data[id].mode;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[5] = 0xFF;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[12] = (uint8_t)Send_T;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[13] = (uint8_t)(Send_T >> 8);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[14] = (uint8_t)Send_W;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[15] = (uint8_t)(Send_W >> 8);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[16] = (uint8_t)Send_Pos;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[17] = (uint8_t)(Send_Pos >> 8);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[18] = (uint8_t)(Send_Pos >> 16);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[19] = (uint8_t)(Send_Pos >> 24);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[20] = (uint8_t)Send_KP;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[21] = (uint8_t)(Send_KP >> 8);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[22] = (uint8_t)Send_KW;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[23] = (uint8_t)(Send_KW >> 8);

    unsigned long CRCData = crc32_core((uint32_t *)UntreeA1_Motor_data[id].Send_data.Tx_buff, 7);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[30] = (uint8_t)CRCData;
    UntreeA1_Motor_data[id].Send_data.Tx_buff[31] = (uint8_t)(CRCData >> 8);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[32] = (uint8_t)(CRCData >> 16);
    UntreeA1_Motor_data[id].Send_data.Tx_buff[33] = (uint8_t)(CRCData >> 24);

    if (id <= Uart2_2)
        HAL_UART_Transmit_DMA(&huart2, UntreeA1_Motor_data[id].Send_data.Tx_buff, 34);
    else
        HAL_UART_Transmit_DMA(&huart3, UntreeA1_Motor_data[id].Send_data.Tx_buff, 34);
}

/**
 * @brief 宇树电机设置参数
 *
 * @param id 电机ID
 * @param T 力矩
 * @param W 速度
 * @param pos 位置
 * @param K_P
 * @param K_W
 */
void UntreeA1_set(uint8_t id, float T, float W, float K_W, float pos, float K_P)
{
    UntreeA1_Motor_data[id].Set.T = T / 9.1f;
    UntreeA1_Motor_data[id].Set.W = W;
    UntreeA1_Motor_data[id].Set.Pos = pos;
    UntreeA1_Motor_data[id].Set.K_P = K_P;
    UntreeA1_Motor_data[id].Set.K_W = K_W;
}

#endif // USE_UNTREE_MOTOR

#if (USE_LINGKONG_MOTOR == 1)

LK_motor_data_s LK_motor_data[3][2];

/**
 * @brief 瓴控电机初始化
 *
 * @param motor_id 电机ID
 */
void LK_Motor_init(LK_Can_id_e motor_id)
{
    uint8_t cantype = motor_id / 2; // 获得电机所在can路
    uint8_t canid = motor_id % 2;   // 得到电机ID值；

    memset(&LK_motor_data[cantype][canid], 0, sizeof(LK_motor_data_s));
    LK_motor_data[cantype][canid].id = canid + 1; // 设置ID
}

/**
 * @brief 瓴控电机设置力矩
 *
 * @param motor_id 电机ID
 * @param T 设置力矩
 */
void LK_Motor_set(LK_Can_id_e motor_id, float T)
{
    uint8_t cantype = motor_id / 2; // 获得电机所在can路
    uint8_t canid = motor_id % 2;   // 得到电机ID值

    LK_motor_data[cantype][canid].T_set = T;
}

/**
 * @brief 瓴控电机发送控制数据
 *
 * @param motor_id 电机ID
 */
void LK_Motor_send_data(LK_Can_id_e motor_id)
{
    // 1. 定义常量
    const float torque_constant = 0.32f; // 转矩系数：0.32 N·m/A
    const float current_max = 32.0f;     // 最大电流 ±32A
    const float control_max = 2000.0f;   // 控制值范围 ±2000

    uint8_t cantype = motor_id / 2; // 获得电机所在can路
    uint8_t canid = motor_id % 2;   // 得到电机ID值
    uint8_t data[8];

    // 2. 初始化数据数组
    data[0] = 0xA1; // 命令字节
    data[1] = 0x00; // NULL
    data[2] = 0x00; // NULL
    data[3] = 0x00; // NULL
    data[4] = 0x00; // 转矩电流控制值低字节 (暂存)
    data[5] = 0x00; // 转矩电流控制值高字节 (暂存)
    data[6] = 0x00; // NULL
    data[7] = 0x00; // NULL

    // 3. 计算所需电流值 (A)
    float current_A = LK_motor_data[cantype][canid].T_set / torque_constant;

    // 4. 电流限幅 [-32A, 32A]
    if (current_A > current_max)
        current_A = current_max;
    if (current_A < -current_max)
        current_A = -current_max;

    // 5. 电流值转换为控制值 [-2000, 2000]
    int16_t iqControl = (int16_t)roundf(current_A * (control_max / current_max));

    // 6. 拆分控制值为高低字节 (小端序)
    data[4] = (uint8_t)(iqControl & 0xFF);        // 低字节
    data[5] = (uint8_t)((iqControl >> 8) & 0xFF); // 高字节

    if (cantype == 0)
        Fdcanx_SendData(&hfdcan1, canid + 1 + 0x140, data, 8);
    if (cantype == 1)
        Fdcanx_SendData(&hfdcan2, canid + 1 + 0x140, data, 8);
    if (cantype == 2)
        Fdcanx_SendData(&hfdcan3, canid + 1 + 0x140, data, 8);
}

/**
 * @brief 瓴控向电机索取总角度数据
 *
 * @param motor_id 电机ID
 */
void LK_ask_for_angle_cnt(LK_Can_id_e motor_id)
{
    uint8_t cantype = motor_id / 2; // 获得电机所在can路
    uint8_t canid = motor_id % 2;   // 得到电机ID值
    uint8_t data[8];

    // 2. 初始化数据数组
    data[0] = 0x92; // 命令字节
    data[1] = 0x00; // NULL
    data[2] = 0x00; // NULL
    data[3] = 0x00; // NULL
    data[4] = 0x00; // 转矩电流控制值低字节 (暂存)
    data[5] = 0x00; // 转矩电流控制值高字节 (暂存)
    data[6] = 0x00; // NULL
    data[7] = 0x00; // NULL

    if (cantype == 0)
        Fdcanx_SendData(&hfdcan1, canid + 1 + 0x140, data, 8);
    if (cantype == 1)
        Fdcanx_SendData(&hfdcan2, canid + 1 + 0x140, data, 8);
    if (cantype == 2)
        Fdcanx_SendData(&hfdcan3, canid + 1 + 0x140, data, 8);
}

/**
 * @brief 瓴控电机解析数据
 *
 * @param hfdcan can通讯句柄
 * @param id 电机ID
 * @param data 数据
 */
void LK_Motor_decode_data(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id < 0x140 || id > 0x150)
        return;
    uint8_t cantype; // 获得电机所在can路
    uint8_t canid;   // 得到电机ID值；
    id--;
    if (hfdcan == &hfdcan1)
    { // 瓴控电机反馈ID
        cantype = 0;
    }
    else if (hfdcan == &hfdcan2)
    {
        cantype = 1;
    }
    else if (hfdcan == &hfdcan3)
    {
        cantype = 2;
    }
    canid = id - 0x140;
    switch (*((uint8_t *)data))
    {
    case 0xA1: // 温度，转矩，速度，角度数据
        LK_motor_data[cantype][canid].temperature = (int8_t)data[1];
        LK_motor_data[cantype][canid].T_now = (int16_t)(((uint16_t)data[3] << 8) | (uint16_t)data[2]) * (32.0f / 2000.0f) * 0.32f;
        LK_motor_data[cantype][canid].W = (int16_t)(((uint16_t)data[5] << 8) | (uint16_t)data[4]) * PI / 180.0f;
        LK_motor_data[cantype][canid].last_pos = LK_motor_data[cantype][canid].pos;
        LK_motor_data[cantype][canid].pos = (((uint16_t)data[7] << 8) | (uint16_t)data[6]) * 2 * PI / 65535;
        // count cnt
        if (LK_motor_data[cantype][canid].last_pos > 1.5f * PI && LK_motor_data[cantype][canid].pos < 0.5 * PI)
            LK_motor_data[cantype][canid].Angle_cnt += ((PI - LK_motor_data[cantype][canid].last_pos) + LK_motor_data[cantype][canid].pos);
        else if (LK_motor_data[cantype][canid].last_pos < 0.5 * PI && LK_motor_data[cantype][canid].pos > 1.5f * PI)
            LK_motor_data[cantype][canid].Angle_cnt -= ((PI - LK_motor_data[cantype][canid].pos) + LK_motor_data[cantype][canid].last_pos);
        else
            LK_motor_data[cantype][canid].Angle_cnt += (LK_motor_data[cantype][canid].pos - LK_motor_data[cantype][canid].last_pos);
        // LK_motor_data[cantype][canid].T_now = (float)((*((int8_t *)data + 3) << 8) | *((int8_t *)data + 2)) * (32.0f / 2000.0f) * 0.32f;
        // LK_motor_data[cantype][canid].W = -(float)((*((int8_t *)data + 5) << 8) | *((int8_t *)data + 4)) * PI / 180.0f;
        // LK_motor_data[cantype][canid].pos = (float)((*((int8_t *)data + 7) << 8) | *((int8_t *)data + 6)) * 2 * PI / 16383;
        break;
    case 0x92: // 总角度数据
        // LK_motor_data[cantype][canid].Angle_cnt = (((uint64_t)data[7] << 8 * 7) |
        //                                            ((uint64_t)data[6] << 8 * 6) |
        //                                            ((uint64_t)data[5] << 8 * 5) |
        //                                            ((uint64_t)data[4] << 8 * 4) |
        //                                            ((uint64_t)data[3] << 8 * 3) |
        //                                            ((uint64_t)data[2] << 8 * 2) |
        //                                            ((uint64_t)data[1] << 8 * 1)) /
        //                                           256 / 100.0f / 180.0f * PI;
        break;
    default:
        break;
    }
}
#endif // USE_LINGKONG_MOTOR
