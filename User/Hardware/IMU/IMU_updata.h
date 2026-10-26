/**
 * @file IMU_Updata.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "struct_typedef.h"
#include "Kalman_Filter_c.h"
#include "hipnuc_dec.h"
#include "dm_imu.h"

#ifndef __IMU_UPDATA__
#define __IMU_UPDATA__

#define IMU_UPDATA_TIME (0.001f) // TIM13更新周期，单位s；与AHRS和速度积分共用
#define IMU_GRAVITY (9.80665f)   // 标定前的默认重力加速度，单位m/s^2
#define IMU_GRAVITY_SAMPLE_COUNT (3000U) // 1kHz下连续采样3s，更新频率变化时同步调整
#define IMU_GRAVITY_GYRO_LIMIT (0.1f) // 标定时允许的角速度模长，单位rad/s
#define IMU_GRAVITY_ACCEL_LIMIT (0.2f) // 相对窗口均值的加速度向量偏差上限，单位m/s^2
#define IMU_GRAVITY_GYRO_DIFF_LIMIT (0.02f) // 相对窗口角速度均值的偏差上限，单位rad/s
#define IMU_GRAVITY_LEVEL_LIMIT (1.0f) // 标定时水平加速度模长上限，仅排除明显倾斜，仍需人工放平
#define IMU_ACCEL_DEADZONE (0.0f) // 去重力后的加速度死区，单位m/s^2；默认关闭，按静止噪声标定
#define IMU_POSITION_X (0.217f) // 旋转中心指向IMU的位置，使用IMU自身右手系，单位m
#define IMU_POSITION_Y (0.0f)
#define IMU_POSITION_Z (0.0f)

struct IMU_t
{
	fp32 gyro[3];  // 零偏补偿后的角速度，IMU自身右手系，rad/s
	fp32 accel[3]; // 静止基线补偿后的加速度计值，仍含重力，IMU自身右手系，m/s^2
	fp32 gyro_raw[3];  // 传感器原始角速度，rad/s
	fp32 accel_raw[3]; // 传感器原始加速度，m/s^2
	fp32 gyro_offset[3];  // 上电3s角速度均值，rad/s
	fp32 accel_offset[3]; // 水平静止均值-[0,0,gravity]；不是六面标定得到的完整零偏
	fp32 accel_rotation[3]; // alpha叉乘r + omega叉乘(omega叉乘r)，滤波前，m/s^2
	fp32 mag[3];   // 磁力计ut
	fp32 temp;	   // 温度

	// 所有机体系输出均沿用IMU自身右手系，不额外对X/Y取负号
	fp32 accel_linear[3]; // 旋转中心去重力线加速度，IMU自身右手系，单位m/s^2
	fp32 speed[3];        // 旋转中心线速度，IMU自身右手系，单位m/s
	fp32 speed_earth[3];  // 积分状态，AHRS固定参考系（Z向上，水平航向由初始化决定），单位m/s
	uint8_t speed_valid;  // 本拍速度更新有效；不代表无漂移或已完成轮速融合
	fp32 gravity;        // 3s加速度模长平均值，单位m/s^2；gravity_ready为1后使用
	uint16_t gravity_sample_count; // 当前连续有效标定样本数，0~3000
	uint8_t gravity_ready; // 0:等待水平静止标定，1:重力和静止零偏标定完成
	
	float calibration[3];//角速度校准error
	float cali_end[3];

	Attitude_3D_t KF; //卡尔曼滤波结果

	struct
	{
		fp32 q[4];			   //四元数
		fp32 pitch, yaw, roll; //欧拉角
	} Mahony;

	struct
	{
		fp32 q[4];			   //四元数
		fp32 pitch, yaw, roll; //欧拉角
	} madgwick;

	struct
	{
		fp32 q[4];			   //四元数
		fp32 pitch, yaw, roll; //欧拉角
		int8_t err_code;
		fp32 last_yaw;
		fp32 yaw_rad_cnt;
		fp32 yaw_angle_cnt;
	} AHRS;

};

extern struct IMU_t IMU_data;
extern struct IMU_t IMU_data_history[10]; // IMU历史数据结构体
extern hipnuc_raw_t IMU_HI_GIMBAL_data;
extern unsigned  long ulTdleCycleCount;	


// 外部调用
void IMU_Init(void);   // IMU初始化
void IMU_Updata(void); // IMU数据更新
uint8_t IMU_AccelUpdata(fp32 timing_time); // 原始数据标定、零偏/旋转补偿和低通；返回本拍有效性
void IMU_SpeedUpdata(fp32 timing_time); // 四元数去重力并积分XYZ线速度，周期单位s
void IMU_SpeedReset(void); // 已知静止时清零速度；与IMU更新互斥调用
void IMU_GravityUpdata(void); // 每个IMU采样周期调用一次，水平静止3s求重力及零偏
void IMU_GravityReset(void); // 重启重力/零偏标定并清零速度；与IMU更新互斥调用
void MagUpdate(void);  // 磁力计数据更新
void MagZero(void); // 清零地磁计
void Get_Angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);//非调用Lib版本
void Process_IMUData();
float get_history_data(uint8_t yaw_or_pitch,uint32_t history_time);
float get_history_q_data(uint8_t i,uint32_t history_time);//获取历史数据函数
float rad2degree(float a);
float degree2rad(float a);
void IMU_CailProgram(void);
void IMU_Offest(void);
#endif
// end of flie
