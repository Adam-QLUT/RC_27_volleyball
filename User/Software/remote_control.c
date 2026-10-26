#include "remote_control.h"
#include "Global_status.h"
#include "Chassis_omni.h"
#include "ramp_generator.h"


#include "DT7.h"
#include "FSI6X.h"
#include "Power_switch.h"
#include "IMU_updata.h"

#include "Stm32_time.h"
#include "Power_switch.h"
#include "cmsis_os2.h"

#define CHASSIS_MAX_SPEED_X 2.5f              // 底盘x轴最大物理速度  m/s
#define CHASSIS_MAX_SPEED_Y 2.5f              // 底盘y轴最大物理速度  m/s
    // 侧视前保存的yaw角度

RC_ctrl_t RC_data; 
/*----------------------------------- 遥控数据更新 --------------------------------------*/
/**
 * @brief 遥控数据来源于DT7遥控器
 *
 */
void DT7toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = DT7_data.rc.ch[0];
    RC_data.rc.ch[1] = DT7_data.rc.ch[1];
    RC_data.rc.ch[2] = DT7_data.rc.ch[2];
    RC_data.rc.ch[3] = DT7_data.rc.ch[3];
    RC_data.rc.ch[4] = DT7_data.rc.ch[4];
    RC_data.rc.s[0] = DT7_data.rc.s[0];
    RC_data.rc.s[1] = DT7_data.rc.s[1];
    /*键鼠数据 */
    RC_data.key.v = DT7_data.key.v;
    RC_data.mouse.x = DT7_data.mouse.x;
    RC_data.mouse.y = DT7_data.mouse.y;
    RC_data.mouse.z = DT7_data.mouse.z;
    RC_data.mouse.press_l = DT7_data.mouse.press_l;
    RC_data.mouse.press_r = DT7_data.mouse.press_r;
    RC_data.mouse.press_mid = 0;
    DT7_data.online--;
    RC_data.online = DT7_data.online;
}

/**
 * @brief 遥控数据来源于FS_I6X遥控器
 *
 */
void FSI6XtoRCdata()
{
    RC_data.rc.ch[0] = FSI6X_data.CH1;
    RC_data.rc.ch[1] = FSI6X_data.CH2;
    RC_data.rc.ch[2] = FSI6X_data.CH4;
    RC_data.rc.ch[3] = FSI6X_data.CH3;

    for (int i = 0; i < 4; i++) // 死区判断
    {
        if (fabs(RC_data.rc.ch[i]) < 10)
            RC_data.rc.ch[i] = 0;
    }

    if (FSI6X_data.CH8 == FS_DOWN) // LOCK
    {
        RC_data.rc.s[0] = RC_SW_DOWN;
        RC_data.rc.s[1] = RC_SW_DOWN;
    }else if(FSI6X_data.CH8 == FS_UP)
    {
        RC_data.rc.s[1]=RC_SW_MID; 
        RC_data.rc.s[0]=RC_SW_MID;
    }
    
    FSI6X_data.online--;
    RC_data.online = FSI6X_data.online;
}

/**
 * @brief 根据数据来源更新遥控数据
 *
 */
void RCdata_Updater()
{
    if (DT7_data.online >= 0)
      DT7toRCdata();
    else if (FSI6X_data.online >= 0)
      FSI6XtoRCdata();
}

/*----------------------------------- 遥控器控制逻辑 --------------------------------------*/
/**
 * @brief 遥控器控制
 *
 */
void RC_Controller()
{
    if (RC_data.online >= 0)
        RC_data.online--;
    /*控制模式选择*/
    if ((RC_data.rc.s[0] == RC_SW_UP && RC_data.rc.s[1] == RC_SW_DOWN) || (RC_data.online <= 0)) // 右上左下，锁死
        Global.Control.mode = LOCK;
    else
        Global.Control.mode = RC;
    if (Global.Control.mode != RC)
        return;
    /*底盘控制：FSI6X 暂不要求拨杆位置，推摇杆即动*/
    
        Global.Chassis.input.x = (RC_data.rc.ch[1]/ 100.0f);
        Global.Chassis.input.y= -(RC_data.rc.ch[0]/ 100.0f) ;
        Chassis_SetX(Global.Chassis.input.x);
        Chassis_SetY(Global.Chassis.input.y);
    
   
}
/*------------------------------------ Task -------------------------------------*/

/**
 * @brief          遥控任务
 * @param          none
 * @retval         none
 */
void Remote_Tasks(void)
{
    // 遥控数据更新
    RCdata_Updater();
    // 遥控器控制
    RC_Controller();
}