#include "motor.h"
#include <stdlib.h> 

// PWM参数（根据定时器配置调整，16位定时器最大65535，此处设7200适配你的需求）
#define PWM_MAX 7200
#define PWM_MIN -7200
#define MOTOR_DEAD_ZONE 200  // 电机死区值（需实际测试调整，比如100~300）
#define ANGLE_PROTECT_THRESHOLD 60.0f // 倾斜保护阈值（°）

extern TIM_HandleTypeDef htim1;
extern uint8_t stop; // 停止标志位（在control.c中定义）

// 自定义绝对值函数（兼容整型）
int abs_custom(int p) 
{
    return (p > 0) ? p : -p;
}

// 辅助函数：应用死区补偿并限幅（核心逻辑保留，优化注释）
// 输入：原始PID输出PWM
// 输出：补偿死区后且限幅的最终PWM
int Apply_DeadZone_And_Limit(int pwm_in)
{
    int pwm_out = 0;

    if(pwm_in == 0) return 0; // 无输出时直接返回0

    // 1. 死区补偿（解决电机低速不转/抖动问题）
    if(pwm_in > 0)
    {
        pwm_out = pwm_in + MOTOR_DEAD_ZONE; // 正转补偿
    }
    else if(pwm_in < 0)
    {
        pwm_out = pwm_in - MOTOR_DEAD_ZONE; // 反转补偿
    }

    // 2. 最终限幅（防止补偿后超出PWM范围）
    if(pwm_out > PWM_MAX) pwm_out = PWM_MAX;
    if(pwm_out < PWM_MIN) pwm_out = PWM_MIN;

    return pwm_out;
}

// 电机PWM输出函数（修复电机2跟随错误，优化逻辑）
void Load(int moto1, int moto2) // 输入范围：-7200~7200
{
    // 第一步：处理死区和限幅，得到最终PWM值
    int pwm1_final = Apply_DeadZone_And_Limit(moto1);
    int pwm2_final = Apply_DeadZone_And_Limit(moto2);

    // --- 电机 1 控制 (TIM1 Channel 4) ---
    // 设置电机转向
    if(pwm1_final < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    }
    // 设置PWM占空比（取绝对值）
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, abs_custom(pwm1_final));

    // --- 电机 2 控制 (TIM1 Channel 1) ---
    // 设置电机转向
    if(pwm2_final < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
    // 修复：使用pwm2_final而非moto1，确保电机2独立控制
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs_custom(pwm2_final));
}

// PWM限幅函数（与Apply_DeadZone_And_Limit重复，保留以兼容Control调用）
void Limit(int *motoA, int *motoB)
{
    if(*motoA > PWM_MAX) *motoA = PWM_MAX;
    if(*motoA < PWM_MIN) *motoA = PWM_MIN;
    if(*motoB > PWM_MAX) *motoB = PWM_MAX;
    if(*motoB < PWM_MIN) *motoB = PWM_MIN;
}

// 安全停止函数（修复逻辑错误，适配俯仰角保护）
// 输入：当前俯仰角（pitch），而非角度差
void Stop(float current_pitch)
{
    // 1. 检测实际倾斜角度（绝对值>60°触发停止）
    if(abs_custom((int)(current_pitch * 10)) > 600) // 放大10倍保留小数，避免精度丢失
    {
        Load(0, 0);       // 输出0 PWM，停止电机
        stop = 1;         // 标记停止
    }
    else
    {
        stop = 0;         // 角度正常，复位停止标志位
    }
}