#include "zf_common_headfile.h"
#include "my_common.h"

//控制电机速度，可取0-100，若太大可调低，若使用3S电池，占空比应限幅60%，默认10%
int8 duty = 30;	
extern float adc_error;
float bigger = 2.0;
extern bool stop_car;
int16_t target_speed = 25;
extern int16 encoder_data_1;
extern int16 encoder_data_2;

// 单个电机的增量式PID结构体定义
typedef struct {
    // PID参数
    float KP;          // 比例系数
    float KI;          // 积分系数
    float KD;          // 微分系数

    // 速度目标值与实际值
    int16_t target_speed;// 目标速度
    int16_t actual_speed;// 当前实际速度
    
    // 误差缓存（e(k), e(k-1), e(k-2)）
    int16_t err[3];      // err[0]=e(k), err[1]=e(k-1), err[2]=e(k-2)
    
    // 输出限制
    int8 out_max;     // 输出最大值
    int8 out_min;     // 输出最小值
    
    // 当前PID输出（PWM占空比）
    int8 output;     
    
} IncPID_Motor_TypeDef;


// 增量式PID初始化函数
void IncPID_Motor_Init(IncPID_Motor_TypeDef *pid, float KP, float KI, float KD,int16_t init_target) {
    if (pid == NULL) return;
    
    // 初始化PID参数
    pid->KP = KP;
    pid->KI = KI;
    pid->KD = KD;
    
    // 初始化速度值
    pid->target_speed = init_target;
    pid->actual_speed = 0;
    
    // 初始化误差缓存（清零）
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f;
    
    // 初始化PID输出
    pid->output = 0;
    pid->out_max=60;
    pid->out_min = 0;
}

// 增量式PID计算函数（每次采样周期调用一次）
// 参数：pid结构体指针、当前实际速度
// 返回：最终PID输出（PWM占空比）
float IncPID_Motor_Calc(IncPID_Motor_TypeDef *pid, int16_t actual_speed) {

    if (pid == NULL) return 0.0f;
    
    // 1. 更新实际速度
    pid->actual_speed = actual_speed;
    
    // 2. 计算当前误差e(k)
    pid->err[0] = (float)pid->target_speed - (float)pid->actual_speed;
    
    // 3. 计算增量Δu(k)（增量式PID核心公式）
    float delta_u = (float)(pid->KP * (pid->err[0] - pid->err[1]) 
                  + pid->KI * pid->err[0] 
                  + pid->KD * (pid->err[0] - 2*pid->err[1] + pid->err[2]));
    
    // 4. 更新最终输出（累加增量+限幅）
    pid->output += delta_u;
    if(pid->output>pid->out_max)
    {
        pid->output = pid->out_max;
    }
     if(pid->output<pid->out_min)
    {
        pid->output = pid->out_min;
    }
    
    // 5. 误差缓存移位（e(k-1)->e(k-2)，e(k)->e(k-1)）
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    
    return pid->output;
}

// 设置电机目标速度函数
void IncPID_Motor_SetTarget(IncPID_Motor_TypeDef *pid, int16_t target) {
    if (pid == NULL) return;
    pid->target_speed = target;
}

IncPID_Motor_TypeDef speed_left,speed_right;

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
 
    if(adc_error<-0.3){
    IncPID_Motor_SetTarget(&speed_left,target_speed*1.099);
    IncPID_Motor_SetTarget(&speed_right,target_speed*0.901);}
    else if(adc_error>0.3){
      IncPID_Motor_SetTarget(&speed_left,target_speed*0.901);
      IncPID_Motor_SetTarget(&speed_right,target_speed*1.099);
    }
    IncPID_Motor_Calc(&speed_left,-encoder_data_1);
    IncPID_Motor_Calc(&speed_right,encoder_data_2);
    
    int8 duty_right = speed_left.output;
    int8 duty_left = speed_right.output;
	//占空比限幅
	if(duty_left > MAX_PWM_DUTY)duty_left = MAX_PWM_DUTY;
    if(duty_left < 10) duty_left = 10;
    if(duty_right > MAX_PWM_DUTY)duty_right = MAX_PWM_DUTY;
    if(duty_right < 10) duty_right = 10;
    if(stop_car){
            duty_right=0;
            duty_left=0;
            }             
    gpio_set_level(MOTOR1_DIR, GPIO_LOW);   
	gpio_set_level(MOTOR2_DIR, GPIO_LOW);         
                         
	pwm_set_duty(MOTOR1_PWM, duty_left * (PWM_DUTY_MAX / 100));                   // 计算占空比
	pwm_set_duty(MOTOR2_PWM, duty_right * (PWM_DUTY_MAX / 100));                   // 计算占空比
    
}


