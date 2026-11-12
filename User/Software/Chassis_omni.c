#include "Chassis_omni.h"
#include "Global_status.h"
#include "Stm32_time.h"
#include "User_math.h"
#include "robot_param.h"

Chassis_t Chassis;

/*---------------------------------- 角度限制 ----------------------------------*/

/**
 * @brief          多圈角度限制
 * @param          angle:要化简的角度
 * @retval         angle:化简后的角度
 */
float Chassis_AngleLimit(float angle)
{
    uint32_t mul = fabs(angle) / 180.0f;
    if (angle > 180.0f)
    {
        if (mul % 2 == 1) // 处于-180度
            angle -= (mul + 1) * 180.0f;
        else // 处于180度
            angle -= mul * 180.0f;
    }
    if (angle < -180.0f)
    {
        if (mul % 2 == 1) // 处于180度
            angle += (mul + 1) * 180.0f;
        else // 处于-180度
            angle += mul * 180.0f;
    }
    return angle;
}

/*----------------------------------- 初始化 -----------------------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Chassis_Init()
{
    // 底盘电机初始化
    CHASSISMotor_init(DJI_M3508, WHEEL_FL);
    CHASSISMotor_init(DJI_M3508, WHEEL_FR);
    CHASSISMotor_init(DJI_M3508, WHEEL_BL);
    CHASSISMotor_init(DJI_M3508, WHEEL_BR);

    // PID初始化
    /*PID速度环初始化*/
    PID_Set(&Chassis.chassis_speed_pid_FL, 100.0f, 0.0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_speed_pid_FR, 100.0f, 0.0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_speed_pid_BL, 100.0f, 0.0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_speed_pid_BR, 100.0f, 0.0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    
    /*底盘力控PID*/
    PID_Set(&Chassis.chassis_T_pid_x, 30.0f, 0.0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_T_pid_y, 30.0f, 0.0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    PID_Set(&Chassis.chassis_T_pid_w, 30.0f, 0.0, 0, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
   
    // 底盘运动斜坡
    RampGenerator_Init(&Chassis.Vx_ramp, CHASSIS_TASK_TIME, 40, 40, 4);
    RampGenerator_Init(&Chassis.Vy_ramp, CHASSIS_TASK_TIME, 40, 40, 4);
    RampGenerator_Init(&Chassis.Vw_ramp, CHASSIS_TASK_TIME, 300, 300, 4);


}

/*--------------------------------- 状态量更新 ---------------------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */

void Chassis_Updater()
{

    /*---------------状态量更新----------------*/
    // 轮速(rad/s)
    Chassis.speed_now_FL = CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    Chassis.speed_now_FR = CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    Chassis.speed_now_BL = CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    Chassis.speed_now_BR = CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S;
    // PID反馈轮速(RPM)
    Chassis.speed_pid_fb_FL = CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO;
    Chassis.speed_pid_fb_FR = CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO;
    Chassis.speed_pid_fb_BL = CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO;
    Chassis.speed_pid_fb_BR = CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO;
    

    /*---------------目标量更新----------------*/
    // 车速
    Chassis.Vx_set = Global.Chassis.input.x;
    Chassis.Vy_set = Global.Chassis.input.y;
    Chassis.W_set = Global.Chassis.input.r;
}

/*----------------------------------- 解算 -------------------------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
#define K (0.0001f)
void Chassis_Calculater()
{
    // 底盘电机前馈力矩定义
    float T_FR, T_FL, T_BR, T_BL; // 转动解算的电机前馈力矩
    float F_FR, F_FL, F_BR, F_BL; // 移动解算的电机前馈力矩
    float Fx, Fy, T_w;            // PID计算的底盘前馈力矩
    // 输出最大电流
    float max_current = 0;

    /*-------速度解算-------*/
    // 目标车速解算
    if (Global.Chassis.mode == FLOW)
    {
        float Vx = Chassis.Vx_set;                                                                                    
        float Vy = Chassis.Vy_set;
        float yaw = Chassis.chassis_yaw_angle; // 已是弧度
        Chassis.Vx_set = Vx * cosf(yaw) - Vy * sinf(yaw);
        Chassis.Vy_set = Vx * sinf(yaw) + Vy * cosf(yaw);
        // yaw 为云台相对底盘的逆时针角度；yaw > 0 时底盘应以 W > 0 跟随。
        Chassis.W_set = -PID_Cal(&Chassis.chassis_follow_pid, yaw, 0.0f);
    }
    else if (Global.Chassis.mode == SPIN_P || Global.Chassis.mode == SPIN_N)
    {
        float Vx = Chassis.Vx_set;
        float Vy = Chassis.Vy_set;
        // 底盘逆时针旋转时，相对 yaw 减小；按反馈角速度补偿延迟。
        float angle = Chassis.chassis_yaw_angle - K * Chassis.W_now;
        Chassis.Vx_set = Vx * cosf(angle) - Vy * sinf(angle);
        Chassis.Vy_set = Vx * sinf(angle) + Vy * cosf(angle);
        // SPIN_P：俯视逆时针；SPIN_N：俯视顺时针。
        Chassis.W_set = (Global.Chassis.mode == SPIN_P) ? (60 * RPM_TO_RAD_S) : (-60 * RPM_TO_RAD_S);
    }
    else if (Global.Chassis.mode == NO_FOLLOW)
    {
        // 移动方向跟随云台视角，但底盘不自转跟随
        float Vx = Chassis.Vx_set;
        float Vy = Chassis.Vy_set;
        float yaw = Chassis.chassis_yaw_angle;
        Chassis.Vx_set = Vx * cosf(yaw) - Vy * sinf(yaw);
        Chassis.Vy_set = Vx * sinf(yaw) + Vy * cosf(yaw);
        Chassis.W_set = 0;
    }

    // 逆解目标轮速
    Chassis.speed_set_FL = RAD_S_TO_RPM * IK_WHEEL_FL(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);
    Chassis.speed_set_FR = RAD_S_TO_RPM * IK_WHEEL_FR(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);
    Chassis.speed_set_BL = RAD_S_TO_RPM * IK_WHEEL_BL(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);
    Chassis.speed_set_BR = RAD_S_TO_RPM * IK_WHEEL_BR(Chassis.Vx_set, Chassis.Vy_set, Chassis.W_set);

    // 正解当前车速
    Chassis.Vx_now = FK_VX(Chassis.speed_now_FL, Chassis.speed_now_FR, Chassis.speed_now_BL, Chassis.speed_now_BR);
    Chassis.Vy_now = FK_VY(Chassis.speed_now_FL, Chassis.speed_now_FR, Chassis.speed_now_BL, Chassis.speed_now_BR);
    Chassis.W_now = FK_OMEGA(Chassis.speed_now_FL, Chassis.speed_now_FR, Chassis.speed_now_BL, Chassis.speed_now_BR);

    /*-------力矩解算-------*/
    Fx = PID_Cal(&Chassis.chassis_T_pid_x, Chassis.Vx_now, Chassis.Vx_set);
    Fy = PID_Cal(&Chassis.chassis_T_pid_y, Chassis.Vy_now, Chassis.Vy_set);
    T_w = PID_Cal(&Chassis.chassis_T_pid_w, Chassis.W_now, Chassis.W_set);
    T_FR = IK_WHEEL_FR_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_FL = IK_WHEEL_FL_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_BR = IK_WHEEL_BR_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_BL = IK_WHEEL_BL_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_FR = IK_WHEEL_FR_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_FL = IK_WHEEL_FL_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_BR = IK_WHEEL_BR_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_BL = IK_WHEEL_BL_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;

    /*-------电流计算-------*/
    Chassis.current[0] = (T_FL + F_FL) + PID_Cal(&Chassis.chassis_speed_pid_FL, CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO, Chassis.speed_set_FL);
    Chassis.current[1] = (T_FR + F_FR) + PID_Cal(&Chassis.chassis_speed_pid_FR, CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO, Chassis.speed_set_FR);
    Chassis.current[2] = (T_BL + F_BL) + PID_Cal(&Chassis.chassis_speed_pid_BL, CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO, Chassis.speed_set_BL);
    Chassis.current[3] = (T_BR + F_BR) + PID_Cal(&Chassis.chassis_speed_pid_BR, CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO, Chassis.speed_set_BR);
    //统一限幅
    for (int i = 0; i < 4; i++)
    { // 求最大值
        if (fabsf(Chassis.current[i]) > max_current)
            max_current = fabsf(Chassis.current[i]);
    }
    if (max_current > MAX_CURRENT)
        for (int j = 0; j < 4; j++)
        { // 限幅
            Chassis.current[j] *= (MAX_CURRENT / max_current);
        }
}

/*----------------------------------- 控制 -------------------------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Chassis_Controller()
{
    float Plimit = 1.0f;
    if (Global.Control.mode != LOCK)
    {
        CHASSISMotor_set(Plimit * Chassis.current[0], WHEEL_FL);
        CHASSISMotor_set(Plimit * Chassis.current[1], WHEEL_FR);
        CHASSISMotor_set(Plimit * Chassis.current[2], WHEEL_BL);
        CHASSISMotor_set(Plimit * Chassis.current[3], WHEEL_BR);
    }
    else
    {
        CHASSISMotor_set(0, WHEEL_FL);
        CHASSISMotor_set(0, WHEEL_FR);
        CHASSISMotor_set(0, WHEEL_BL);
        CHASSISMotor_set(0, WHEEL_BR);
    }
}

/*----------------------------------- 任务 -------------------------------------*/

/**
 * @brief          底盘任务
 * @param          none
 * @retval         none
 */
void Chassis_Tasks(void)
{

    // 底盘数据更新
    Chassis_Updater();
    // 底盘运动解算
    Chassis_Calculater();
    // 底盘电机控制
    Chassis_Controller();
}

/*--------------------------------- 目标值设置 ---------------------------------*/

// @brief 设置底盘水平移动速度

void Chassis_SetX(float x)
{
    //斜坡规划暂时旁路，速度指令直接交给底盘PID。
    RampGenerator_SetTarget(&Chassis.Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Chassis.Vx_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Chassis.Vx_ramp, 0.0f);
    // Chassis.Vx_set = x;
}

// @brief 设置底盘竖直移动速度

void Chassis_SetY(float y)
{
    //斜坡规划暂时旁路，速度指令直接交给底盘PID。
    RampGenerator_SetTarget(&Chassis.Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Chassis.Vy_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Chassis.Vy_ramp, 0.0f);
    // Chassis.Vy_set = y;
}

// @brief 设置底盘角速度

void Chassis_SetR(float r)
{
    Global.Chassis.input.r = r;
}

// @brief 设置斜坡规划器加速度

void Chassis_SetAccel(float acc)
{
    // 斜坡规划暂时旁路，此参数当前不生效。
    // RampGenerator_SetAccel(&Chassis.Vx_ramp, acc);
    // RampGenerator_SetAccel(&Chassis.Vy_ramp, acc);
    (void)acc;
}

