#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

//传感器数据变量
int Encoder_Left,Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;

//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;	
float last_error;
float Med_Angle=0.0f;//平衡时角度值偏移量（机械中值）
//参数
float Vertical_Kp=-400.0f,Vertical_Kd=-2.20f;			//直立环 数量级（Kp：0~1000、Kd：0~10）
float Velocity_Kp=65.0f,Velocity_Ki=-0.0325f;		  //速度环 数量级（Kp：0~1）
float Turn_Kp = 80.0f,Turn_Kd = 2.0f;             //转向环 

uint8_t stop;

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;
extern uint8_t Fore,Back,Left,Right;

#define GYRO_SCALE 16.4f // 陀螺仪换算系数（±2000°/s量程）
#define ALPHA 0.94f // 互补滤波系数

//直立环PD控制器
//输入：期望角度、真实角度、角速度
int Vertical(float target_angle, float current_angle, short gyro_y)
{
    int pd_out;
    // 1. 互补滤波融合角度（仅平滑反馈值）
    static float filtered_angle = 0.0f;
    filtered_angle = ALPHA * filtered_angle + (1 - ALPHA) * current_angle;
    
    // 2. PD核心计算（仅在函数内做一次陀螺仪单位转换）
    float angle_error = target_angle - filtered_angle;
    float gyro_y_dps = (float)gyro_y / GYRO_SCALE; // 转换为°/s
    pd_out = (int)(Vertical_Kp * angle_error + Vertical_Kd * gyro_y_dps);
    
    return pd_out;
}

//速度环
int Velocity(int target_speed, int left_speed, int right_speed)
{
		float speed_error,Lowout;
		float Alpha = 0.7f;
  	static float Lowout_last = 0.0f;
	  static float integral = 0.0f;
    //计算平均速度
    int avg_speed = (left_speed + right_speed) / 2;
    //速度偏差
    speed_error =target_speed  - avg_speed;
		//对速度差进行低通滤波
		Lowout = (1-Alpha)*speed_error+Alpha*Lowout_last;
		Lowout_last = Lowout * Velocity_Ki;
		
	
    integral += Lowout;
    
    integral = (integral > 1.5) ? 1.5f : (integral < -1.50f ? -1.50f : integral);
		
    int pid_out =(int)(Velocity_Kp*Lowout + Velocity_Ki *integral);

		return pid_out;
}

//转向环
int Turn(short gyro_z , int target_turn)
{
	  int pd_out;
    // roll角是转向的核心输入
    float roll_error = (float)target_turn - roll; // target_turn默认0，即追0°roll角
    float gyro_z_dps = (float)gyro_z / GYRO_SCALE;
    
    // PD计算：Turn_Kp决定小角度的转速，越大转得越快
    pd_out = (int)(Turn_Kp * roll_error + Turn_Kd * gyro_z_dps);
    
    return pd_out;
	
}


void Control(void)	//每隔10ms调用一次
{
	int PWM_out;
	//1、读取编码器和陀螺仪的数据
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	
	//2、速度环计算
	Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
	
  //3. 直立环计算（核心：用速度环输出修正目标角度，实现位移控制）
  float angle_target = Med_Angle + (float)Velocity_out * 0.35f; 
  Vertical_out = Vertical(angle_target, pitch, gyroy);
	Turn_out=Turn(gyroz,Target_turn);
	PWM_out=Vertical_out;
	MOTO1=PWM_out - Turn_out;
	MOTO2=PWM_out + Turn_out;
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
	Stop(pitch);//安全检测
}
