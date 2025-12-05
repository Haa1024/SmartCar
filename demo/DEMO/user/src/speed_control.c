#include "zf_common_headfile.h"
#include "my_common.h"

//控制电机速度，可取0-100，若太大可调低，若使用3S电池，占空比应限幅60%，默认10%
int8 duty = 30;	
extern float adc_error;
float bigger = 2.0;
extern bool stop_car;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机PWM
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     DIR控制电机方向，duty控制为占空比控制速度，若车轮反转，尝试将对应的DIR设置为低
//				如若MOTOR1对应的电机反转，可以尝试调用gpio_set_level(MOTOR1_DIR, GPIO_LOW)，使得电机按理想方向旋转
//-------------------------------------------------------------------------------------------------------------------
void set_speed_pwm()
{
    int8 duty_left = duty + adc_error*bigger;
    int8 duty_right = duty - adc_error*bigger;
	//占空比限幅
	if(duty_left > MAX_PWM_DUTY)duty_left = MAX_PWM_DUTY;
    if(duty_left < 10) duty_left = 10;
    if(duty_right > MAX_PWM_DUTY)duty_right = MAX_PWM_DUTY;
    if(duty_right < 10) duty_right = 10;
    if(stop_car){
            duty_right=1;
            duty_left=1;
            gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
            gpio_set_level(MOTOR2_DIR, GPIO_HIGH);}             
         gpio_set_level(MOTOR1_DIR, GPIO_LOW);   
	     gpio_set_level(MOTOR2_DIR, GPIO_LOW);         
                         
	pwm_set_duty(MOTOR1_PWM, duty_left * (PWM_DUTY_MAX / 100));                   // 计算占空比
	pwm_set_duty(MOTOR2_PWM, duty_right * (PWM_DUTY_MAX / 100));                   // 计算占空比
    
}

/*
// PID结构体（百分比duty输出，无积分分离）
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    int target;        // 目标速度（整数，如RPM）
    int current;       // 当前速度（整数，如RPM）
    float error;       // 误差（浮点型提升精度）
    float last_error;  // 上一次误差
    float integral;    // 积分项
    float duty;        // 输出duty值（0~100%）
    float duty_max;    // duty上限（固定100）
    float duty_min;    // duty下限（固定0）
    float integral_max;// 积分限幅（防止饱和）
} PID_Duty_Controller;

// PID初始化（duty范围固定0~100%）
void PID_Duty_Init(PID_Duty_Controller *pid, float Kp, float Ki, float Kd, float integral_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->duty_max = 100.0f;
    pid->duty_min = 0.0f;
    pid->integral_max = integral_max;
    pid->target = 0;
    pid->current = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->duty = 0;
}

// PID计算（无积分分离，输出0~100% duty）
float PID_Duty_Compute(PID_Duty_Controller *pid) {
    // 计算误差（整数速度转浮点误差）
    pid->error = (float)(pid->target - pid->current);

    // 比例项
    float p_term = pid->Kp * pid->error;

    // 积分项（无分离，直接累加+限幅）
    pid->integral += pid->error;
    pid->integral = (pid->integral > pid->integral_max) ? pid->integral_max : 
                   (pid->integral < -pid->integral_max) ? -pid->integral_max : pid->integral;
    float i_term = pid->Ki * pid->integral;

    // 微分项（微分先行，基于当前值变化）
    float d_term = pid->Kd * (pid->current - pid->last_error);
    pid->last_error = (float)pid->current;

    // 总输出（百分比duty）+限幅
    pid->duty = p_term + i_term - d_term;
    pid->duty = (pid->duty > pid->duty_max) ? pid->duty_max : 
               (pid->duty < pid->duty_min) ? pid->duty_min : pid->duty;

    return pid->duty;
}

// 全局PID对象（左右轮，百分比duty输出）
PID_Duty_Controller pid_left, pid_right;

// 速度环控制主函数（直接输出百分比duty）
void Speed_Loop_Duty_Control(void) {
    // 外部整数类型速度（编码器采集）
    extern int speed_left_int, speed_right_int; 
    
    // 更新当前速度
    pid_left.current = speed_left_int;
    pid_right.current = speed_right_int;
    
    // 计算百分比duty（0~100）
    float duty_left = PID_Duty_Compute(&pid_left);
    float duty_right = PID_Duty_Compute(&pid_right);
    
    // 映射duty到PWM硬件值（示例：0~100 → 0~255）
    int pwm_left = (int)(duty_left / 100 * 255);
    int pwm_right = (int)(duty_right / 100 * 255);
    
    // 驱动电机（替换为实际PWM输出函数）
    Motor_Set_PWM(LEFT_MOTOR, pwm_left);
    Motor_Set_PWM(RIGHT_MOTOR, pwm_right);
}

// 初始化示例
void PID_Config(void) {
    // 调参建议：Kp=1.5~3，Ki=0.05~0.2，积分限幅=30~80
    PID_Duty_Init(&pid_left, 2.0f, 0.1f, 0.05f, 50.0f);
    PID_Duty_Init(&pid_right, 2.0f, 0.1f, 0.05f, 50.0f);
    
    // 设置目标速度（整数）
    pid_left.target = 30;  // 30RPM
    pid_right.target = 30;
}
*/