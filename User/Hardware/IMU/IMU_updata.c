/**
 * @file IMU_Updata.c
 * @author sethome
 * @brief IMU updata info
 * @version 0.1
 * @date 2022-11-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "pid.h"
#include "IMU_updata.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"

#include "fdacoefs.h"

#include "User_math.h"
#include "MahonyAHRS.h"
#include "madgwick.h"
#include "AHRS.h"
#include "bsxlite_interface.h"
#include "Vofa_Justfloat_Send.h"

#include "tim.h"
#include "Stm32_time.h"

// made by sethome
// 使用TIM14定频更新

// 陀螺仪温度PID
pid_t IMU_tempure_pid;

// 加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
static uint8_t first_temperate;

// 固定参考系下的上一拍线加速度，用于梯形积分
static fp32 accel_earth_last[3];
static uint8_t speed_started;
static fp32 gravity_accel_mean[3]; // 重力标定窗口内的原始加速度均值，用于检查晃动
static fp32 gravity_gyro_mean[3];
static fp32 gyro_last[3]; // 旋转补偿使用上一拍已校正角速度
static uint8_t rotation_started;
static uint8_t accel_filter_started;

unsigned long ulTdleCycleCount = 0UL;

// 定时器变量
extern TIM_HandleTypeDef htim3;

struct IMU_t IMU_data;			   // IMU数据结构体
struct IMU_t IMU_data_history[10]; // IMU历史数据结构体

hipnuc_raw_t IMU_HI_GIMBAL_data;

// 内部调用
void IMU_HeatSet(uint16_t ccr); // 加热电阻PWM占空比
void shiftArray(float arr[], int size);
static void IMU_TempControl(fp32 temp);

// 四元数转为欧拉角 非调用lib版本，勿删
void Get_Angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
	*yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
	*pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
	*roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

// 初始化
void IMU_Init()
{
	BMI088_init(); // 陀螺仪
	BMI088_read(IMU_data.gyro_raw, IMU_data.accel_raw, &IMU_data.temp);
	// 用首帧初始化滤波器，避免从0爬升到重力加速度时产生虚假的速度
	for (int i = 0; i < 3; i++)
	{
		IMU_data.gyro[i] = IMU_data.gyro_raw[i];
		IMU_data.accel[i] = IMU_data.accel_raw[i];
		accel_fliter_1[i] = IMU_data.accel[i];
		accel_fliter_2[i] = IMU_data.accel[i];
		accel_fliter_3[i] = IMU_data.accel[i];
	}
	IMU_GravityReset();

	IMU_data.Mahony.q[0] = 1.0f;
	IMU_data.Mahony.q[1] = 0.0f;
	IMU_data.Mahony.q[2] = 0.0f;
	IMU_data.Mahony.q[3] = 0.0f;

	IMU_data.madgwick.q[0] = 1.0f;
	IMU_data.madgwick.q[1] = 0.0f;
	IMU_data.madgwick.q[2] = 0.0f;
	IMU_data.madgwick.q[3] = 0.0f;

	AHRS_init(IMU_data.AHRS.q, IMU_data.accel, IMU_data.mag); // AHRS滤波参数

	PID_Set(&IMU_tempure_pid, 2000, 0.2, 0, 0, 4500, 4400);

	HAL_TIM_Base_Start(&htim3); // 加热电阻PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim13); // 使能更新中断，1000HZ

	// 校准加速度（未完成）和角速度计
	// for (int i = 0; i < 1000;)
	// {
	// 	float x_ = IMU_data.gyro[0];
	// 	float y_ = IMU_data.gyro[1];
	// 	float z_ = IMU_data.gyro[2];
	// 	if (fabs(x_) > 1.0f && fabs(y_) > 1.0f && fabs(z_) > 1.0f)
	// 		continue;
	// 	IMU_data.calibration[0] += x_ / 1000.0f;
	// 	IMU_data.calibration[1] += y_ / 1000.0f;
	// 	IMU_data.calibration[2] += z_ / 1000.0f;
	// 	i++;
	// 	HAL_Delay(1);
	// }
}

void Process_IMUData()
{
	// 计算总圈数
	if (IMU_data.AHRS.last_yaw > 3.0f && IMU_data.AHRS.yaw < -3.0f)
		IMU_data.AHRS.yaw_rad_cnt += ((PI - IMU_data.AHRS.last_yaw) + (IMU_data.AHRS.yaw + PI));
	else if (IMU_data.AHRS.last_yaw < -3.0f && IMU_data.AHRS.yaw > 3.0f)
		IMU_data.AHRS.yaw_rad_cnt -= ((PI + IMU_data.AHRS.last_yaw) + (PI - IMU_data.AHRS.yaw));
	else
		IMU_data.AHRS.yaw_rad_cnt += (IMU_data.AHRS.yaw - IMU_data.AHRS.last_yaw);
}
// IMU更新函数
void IMU_Updata() // 1000HZ
{
	//  读取陀螺仪和地磁计信息
	BMI088_read(IMU_data.gyro_raw, IMU_data.accel_raw, &IMU_data.temp);

	// 加热器PID计算
	IMU_TempControl(IMU_data.temp);

	uint8_t gravity_ready = IMU_data.gravity_ready;
	if (!IMU_AccelUpdata(IMU_UPDATA_TIME))
		return;

	// AHRS.lib
	// 如果失败，返回0
	IMU_data.AHRS.err_code = AHRS_update(IMU_data.AHRS.q, IMU_UPDATA_TIME,
										 IMU_data.gyro,
										 accel_fliter_3,
										 IMU_data.mag);
	// 上电先连续静止采样3s；姿态照常更新，完成标定后的下一拍才开始速度积分
	if (gravity_ready && IMU_data.gravity_ready)
		IMU_SpeedUpdata(IMU_UPDATA_TIME);
	else
		IMU_SpeedReset();
	get_angle(IMU_data.AHRS.q, &IMU_data.AHRS.yaw, &IMU_data.AHRS.roll, &IMU_data.AHRS.pitch);

	// Process_IMUData();
	if (IMU_data.AHRS.last_yaw > 3.0f && IMU_data.AHRS.yaw < -3.0f)
		IMU_data.AHRS.yaw_rad_cnt += ((PI - IMU_data.AHRS.last_yaw) + (IMU_data.AHRS.yaw + PI));
	else if (IMU_data.AHRS.last_yaw < -3.0f && IMU_data.AHRS.yaw > 3.0f)
		IMU_data.AHRS.yaw_rad_cnt -= ((PI + IMU_data.AHRS.last_yaw) + (PI - IMU_data.AHRS.yaw));
	else
		IMU_data.AHRS.yaw_rad_cnt += (IMU_data.AHRS.yaw - IMU_data.AHRS.last_yaw);
	IMU_data.AHRS.pitch *= -1;
	IMU_data.AHRS.last_yaw = IMU_data.AHRS.yaw;
	//历史数据更新
	for (int i = 0; i < 8; i++){
		IMU_data_history[i+1]=IMU_data_history[i];
	}
	IMU_data_history[0]=IMU_data;
}

/**
 * @brief          零偏及安装位置补偿，然后对中心加速度做低通滤波
 * @note           r为旋转中心指向IMU的向量，所有计算均在IMU自身右手系。
 *                 a_center = a_imu - alpha x r - omega x (omega x r)。
 *                 先相减再共用低通，避免仅加速度低通而补偿项无延迟。
 *                 角加速度由相邻角速度差分得到，其噪声也经过该低通。
 */
uint8_t IMU_AccelUpdata(fp32 timing_time)
{
	fp32 alpha[3];
	fp32 accel_center[3];
	const fp32 position[3] = {IMU_POSITION_X, IMU_POSITION_Y, IMU_POSITION_Z};
	fp32 gyro_dot_position = 0.0f;
	fp32 gyro_norm = 0.0f;
	uint8_t gravity_ready = IMU_data.gravity_ready;

	uint8_t valid = isfinite(timing_time) && timing_time > 0.0f && timing_time <= 0.02f;
	for (int i = 0; i < 3; i++)
		if (!isfinite(IMU_data.accel_raw[i]) || !isfinite(IMU_data.gyro_raw[i]))
			valid = 0;
	if (!valid)
	{
		if (!gravity_ready)
			IMU_GravityReset();
		IMU_data.speed_valid = 0;
		speed_started = 0;
		rotation_started = 0;
		accel_filter_started = 0;
		return 0;
	}
	if (!gravity_ready)
		IMU_GravityUpdata();
	for (int i = 0; i < 3; i++)
	{
		IMU_data.gyro[i] = IMU_data.gyro_raw[i] - IMU_data.gyro_offset[i];
		IMU_data.accel[i] = IMU_data.accel_raw[i] - IMU_data.accel_offset[i];
		alpha[i] = rotation_started ? (IMU_data.gyro[i] - gyro_last[i]) / timing_time : 0.0f;
		gyro_dot_position += IMU_data.gyro[i] * position[i];
		gyro_norm += IMU_data.gyro[i] * IMU_data.gyro[i];
	}
	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3, k = (i + 2) % 3;
		IMU_data.accel_rotation[i] = IMU_data.gravity_ready ?
			alpha[j] * position[k] - alpha[k] * position[j] +
			IMU_data.gyro[i] * gyro_dot_position - position[i] * gyro_norm : 0.0f;
		accel_center[i] = IMU_data.accel[i] - IMU_data.accel_rotation[i];
		if (!isfinite(accel_center[i]))
		{
			IMU_data.speed_valid = 0;
			speed_started = 0;
			rotation_started = 0;
			accel_filter_started = 0;
			return 0;
		}
	}

	// 零偏生效时清除旧基线的滤波残留；只修正水平姿态，保留既有航向。
	if (!gravity_ready && IMU_data.gravity_ready)
	{
		fp32 *q = IMU_data.AHRS.q;
		fp32 yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		if (!isfinite(yaw))
			yaw = 0.0f;
		q[0] = cosf(yaw * 0.5f);
		q[1] = q[2] = 0.0f;
		q[3] = sinf(yaw * 0.5f);
		accel_filter_started = 0;
	}
	for (int i = 0; i < 3; i++)
	{
		gyro_last[i] = IMU_data.gyro[i];
		if (!accel_filter_started)
			accel_fliter_1[i] = accel_fliter_2[i] = accel_fliter_3[i] = accel_center[i];
		else
		{
			accel_fliter_1[i] = accel_fliter_2[i];
			accel_fliter_2[i] = accel_fliter_3[i];
			// 差分形式保持单位直流增益，避免大重力项的浮点舍入残差被积分。
			// 等价于原二阶滤波（a0+a1+b0=1），不改变滤波器的设计带宽。
			accel_fliter_3[i] = accel_fliter_2[i] -
				fliter_num[1] * (accel_fliter_2[i] - accel_fliter_1[i]) +
				fliter_num[2] * (accel_center[i] - accel_fliter_2[i]);
		}
	}
	rotation_started = IMU_data.gravity_ready;
	accel_filter_started = 1;
	return 1;
}

/**
 * @brief          四元数去重力、固定参考系积分，再转换为IMU自身XYZ线速度
 * @param          timing_time 更新周期，单位s
 * @note           q按(w,x,y,z)，表示传感器到固定参考系的旋转。
 *                 机体系输出沿用IMU自身右手系，不做安装朝向的符号转换。
 *                 初速度默认为0；纯惯性积分会漂移，后续需轮速/外部观测校正。
 *                 输入由IMU_AccelUpdata补偿安装位置，输出为旋转中心速度。
 */
void IMU_SpeedUpdata(fp32 timing_time)
{
	fp32 q[4];
	fp32 rotation[3][3];
	fp32 accel_earth[3] = {0.0f, 0.0f, 0.0f};
	fp32 norm = 0.0f;

	IMU_data.speed_valid = 0;
	if (!IMU_data.gravity_ready)
	{
		IMU_SpeedReset();
		return;
	}
	if (IMU_data.AHRS.err_code != 1 || !isfinite(timing_time) ||
		timing_time <= 0.0f || timing_time > 0.02f ||
		!isfinite(IMU_data.gravity) || IMU_data.gravity < 8.0f || IMU_data.gravity > 12.0f)
	{
		speed_started = 0;
		return;
	}
	for (int i = 0; i < 4; i++)
	{
		q[i] = IMU_data.AHRS.q[i];
		norm += q[i] * q[i];
	}
	if (!isfinite(norm) || norm < 1e-12f)
	{
		speed_started = 0;
		return;
	}
	for (int i = 0; i < 3; i++)
	{
		if (!isfinite(accel_fliter_3[i]))
		{
			speed_started = 0;
			return;
		}
	}
	norm = 1.0f / sqrtf(norm);
	for (int i = 0; i < 4; i++)
		q[i] *= norm;

	// 传感器坐标 -> 固定参考系的旋转矩阵，不使用已做业务符号调整的欧拉角
	rotation[0][0] = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
	rotation[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
	rotation[0][2] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
	rotation[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
	rotation[1][1] = 1.0f - 2.0f * (q[1] * q[1] + q[3] * q[3]);
	rotation[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
	rotation[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	rotation[2][1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	rotation[2][2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);

	for (int i = 0; i < 3; i++)
	{
		// 水平静止时加速度计Z为+g，减去R^T*[0,0,g]后才是线加速度
		fp32 accel = accel_fliter_3[i] - rotation[2][i] * IMU_data.gravity;
		if (fabsf(accel) < IMU_ACCEL_DEADZONE)
			accel = 0.0f;
		IMU_data.accel_linear[i] = accel;
		for (int j = 0; j < 3; j++)
			accel_earth[j] += rotation[j][i] * accel;
	}

	// 在固定坐标轴下积分，避免底盘旋转时直接累加机体系加速度的错误
	for (int i = 0; i < 3; i++)
	{
		if (!speed_started)
			accel_earth_last[i] = accel_earth[i];
		IMU_data.speed_earth[i] += 0.5f * (accel_earth_last[i] + accel_earth[i]) * timing_time;
		accel_earth_last[i] = accel_earth[i];
	}
	for (int i = 0; i < 3; i++)
	{
		fp32 speed = 0.0f;
		for (int j = 0; j < 3; j++)
			speed += rotation[j][i] * IMU_data.speed_earth[j];
		IMU_data.speed[i] = speed;
	}
	speed_started = 1;
	IMU_data.speed_valid = 1;
}

// 已知静止时清零；不可仅凭加速度接近0判断静止，匀速运动也满足这一条件
void IMU_SpeedReset(void)
{
	for (int i = 0; i < 3; i++)
	{
		IMU_data.accel_linear[i] = 0.0f;
		IMU_data.speed[i] = 0.0f;
		IMU_data.speed_earth[i] = 0.0f;
		accel_earth_last[i] = 0.0f;
	}
	speed_started = 0;
	IMU_data.speed_valid = 0;
}

/**
 * @brief          上电连续水平静止3s，计算重力均值、角速度零偏和加速度静止基线
 * @note           必须水平、Z向上；单姿态无法独立辨识真实重力与Z轴零偏。
 *                 accel_offset为mean(accel)-[0,0,mean(|accel|)]，不能代替六面标定。
 *                 检测到明显转动、晃动或无效数据时重新计数。
 *                 IMU不能可靠区分重力与持续平移加速度，标定期间必须保持底盘静止。
 */
void IMU_GravityUpdata(void)
{
	fp32 accel_norm = 0.0f;
	fp32 gyro_norm = 0.0f;
	fp32 accel_diff = 0.0f;
	fp32 gyro_diff = 0.0f;

	if (IMU_data.gravity_ready)
		return;
	IMU_SpeedReset();
	for (int i = 0; i < 3; i++)
	{
		if (!isfinite(IMU_data.accel_raw[i]) || !isfinite(IMU_data.gyro_raw[i]))
		{
			IMU_GravityReset();
			return;
		}
		accel_norm += IMU_data.accel_raw[i] * IMU_data.accel_raw[i];
		gyro_norm += IMU_data.gyro_raw[i] * IMU_data.gyro_raw[i];
		fp32 diff = IMU_data.accel_raw[i] - gravity_accel_mean[i];
		accel_diff += diff * diff;
		diff = IMU_data.gyro_raw[i] - gravity_gyro_mean[i];
		gyro_diff += diff * diff;
	}
	accel_norm = sqrtf(accel_norm);
	if (!isfinite(accel_norm) || accel_norm < 8.0f || accel_norm > 12.0f ||
		gyro_norm > IMU_GRAVITY_GYRO_LIMIT * IMU_GRAVITY_GYRO_LIMIT ||
		IMU_data.accel_raw[2] < 8.0f ||
		IMU_data.accel_raw[0] * IMU_data.accel_raw[0] + IMU_data.accel_raw[1] * IMU_data.accel_raw[1] >
		 IMU_GRAVITY_LEVEL_LIMIT * IMU_GRAVITY_LEVEL_LIMIT ||
		(IMU_data.gravity_sample_count > 0 &&
		 (accel_diff > IMU_GRAVITY_ACCEL_LIMIT * IMU_GRAVITY_ACCEL_LIMIT ||
		  gyro_diff > IMU_GRAVITY_GYRO_DIFF_LIMIT * IMU_GRAVITY_GYRO_DIFF_LIMIT)))
	{
		IMU_GravityReset();
		return;
	}

	// 递推均值等价于求和除以样本数，避免浮点数长时间累加的精度损失
	IMU_data.gravity_sample_count++;
	IMU_data.gravity += (accel_norm - IMU_data.gravity) / IMU_data.gravity_sample_count;
	for (int i = 0; i < 3; i++)
	{
		gravity_accel_mean[i] += (IMU_data.accel_raw[i] - gravity_accel_mean[i]) / IMU_data.gravity_sample_count;
		gravity_gyro_mean[i] += (IMU_data.gyro_raw[i] - gravity_gyro_mean[i]) / IMU_data.gravity_sample_count;
	}
	if (IMU_data.gravity_sample_count >= IMU_GRAVITY_SAMPLE_COUNT)
	{
		for (int i = 0; i < 3; i++)
		{
			IMU_data.gyro_offset[i] = gravity_gyro_mean[i];
			IMU_data.accel_offset[i] = gravity_accel_mean[i] - ((i == 2) ? IMU_data.gravity : 0.0f);
		}
		rotation_started = 0; // 不把零偏生效的阶跃当作角加速度
		IMU_data.gravity_ready = 1;
	}
}

// 标定结果只保存在RAM，每次上电重新测量；单独清零速度不会丢失重力标定值
void IMU_GravityReset(void)
{
	IMU_data.gravity = IMU_GRAVITY;
	IMU_data.gravity_sample_count = 0;
	IMU_data.gravity_ready = 0;
	for (int i = 0; i < 3; i++)
	{
		gravity_accel_mean[i] = 0.0f;
		gravity_gyro_mean[i] = 0.0f;
		IMU_data.gyro_offset[i] = 0.0f;
		IMU_data.accel_offset[i] = 0.0f;
		IMU_data.accel_rotation[i] = 0.0f;
		gyro_last[i] = 0.0f;
	}
	rotation_started = 0;
	accel_filter_started = 0;
	IMU_SpeedReset();
}

float rad2degree(float a)
{
	return a / PI * 180.0f;
}
float degree2rad(float a)
{
	return a / 180.0f * PI;
}

void MagUpdate() // 更新地磁计
{
	// ist8310_read_mag(IMU_data.mag);
}
void MagZero() // 清零地磁计
{
	IMU_data.mag[0] = 0;
	IMU_data.mag[1] = 0;
	IMU_data.mag[2] = 0;
}
// 设定加热的PWM占空比
void IMU_HeatSet(uint16_t ccr)
{
	if (ccr < 0)
	{
		ccr = 0;
	}
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, ccr); // HAL库PWM的CCR设定
													  // TIM10->CCR1 = (pwm);
}

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void IMU_TempControl(fp32 temp)
{
	uint16_t tempPWM;
	static uint8_t temp_constant_time = 0;
	if (first_temperate)
	{
		PID_Cal(&IMU_tempure_pid, temp, 45);

		if (IMU_tempure_pid.total_out < 0.0f)
		{
			IMU_tempure_pid.total_out = 0.0f;
		}
		tempPWM = (uint16_t)IMU_tempure_pid.total_out;
		IMU_HeatSet(tempPWM);
	}
}

void IMU_CailProgram(void) // 陀螺仪校准程序，放在闲置任务里面，需要的时候校准一次算出平均数值加上即可
{
	IMU_data.calibration[0] = 0;
	IMU_data.calibration[1] = 0;
	IMU_data.calibration[2] = 0;
	for (int i = 0; i < 1000;)
	{
		float x_ = IMU_data.gyro[0];
		float y_ = IMU_data.gyro[1];
		float z_ = IMU_data.gyro[2];
		if (fabs(x_) > 1.0f && fabs(y_) > 1.0f && fabs(z_) > 1.0f)
			continue;
		IMU_data.calibration[0] += x_ / 1000.0f;
		IMU_data.calibration[1] += y_ / 1000.0f;
		IMU_data.calibration[2] += z_ / 1000.0f;
		i++;
	}
	if (IMU_data.calibration[0] != 0 && IMU_data.calibration[1] != 0 && IMU_data.calibration[2] != 0)
	{
		IMU_data.cali_end[0] = IMU_data.calibration[0];
		IMU_data.cali_end[1] = IMU_data.calibration[1];
		IMU_data.cali_end[2] = IMU_data.calibration[2];
	}
}

void IMU_Offest()
{
	if (fabs(IMU_data.gyro[0]) > 1.0f && fabs(IMU_data.gyro[1]) > 1.0f && fabs(IMU_data.gyro[2]) > 1.0f)
	{
	}
	else
	{
		for (int j = 0; j < 1000;)
		{
			float x_ = IMU_data.gyro[0];
			float y_ = IMU_data.gyro[1];
			float z_ = IMU_data.gyro[2];

			if (fabs(IMU_data.gyro[0]) > 1.0f && fabs(IMU_data.gyro[1]) > 1.0f && fabs(IMU_data.gyro[2]) > 1.0f)
			{
				IMU_data.gyro[0] -= 0.007288707553;
				IMU_data.gyro[1] -= -0.00267180009;
				IMU_data.gyro[2] -= 0.90244110504;
			}

			IMU_data.calibration[0] += x_ / 1000.0f;
			IMU_data.calibration[1] += y_ / 1000.0f;
			IMU_data.calibration[2] += z_ / 1000.0f;

			j++;
		}
	}

	IMU_data.gyro[0] -= IMU_data.calibration[0];
	IMU_data.gyro[1] -= IMU_data.calibration[1];
	IMU_data.gyro[2] -= IMU_data.calibration[2];

	IMU_data.calibration[0] = 0; //-0.005116222685;
	IMU_data.calibration[1] = 0;
	IMU_data.calibration[2] = 0;
}

float rad_to_deg(float a)
{
	return a / PI * 180.0f;
}

float deg_to_rad(float a)
{
	return a / 180.0f * PI;
}

// end of file
