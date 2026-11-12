#include "Shoot.h"
#include "Global_status.h"
#include "Stm32_time.h"
#include "User_math.h"
#include "robot_param.h"

void Shoot_Init()
{
   LZMotor_init(LZ_CAN_1_1);
   LZMotor_init(LZ_CAN_1_2);
   DMMotor_Init(DM_4310,DM_CAN_3_1);
   
}