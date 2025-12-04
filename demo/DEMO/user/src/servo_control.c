#include "zf_common_headfile.h"
#include "my_common.h"

extern bool reset;

/* PID 结构体 */
struct pid_t{
    float kp, ki, kd;
    float err;       // 本次误差
    float err_last;  // 上次误差
    float integral;  // 积分累加
    float dt;        // 调用周期，单位 s
} ;

/* 舵机 PID 实例 */
struct pid_t servo = {
    .kp = 1.1f,
    .ki = 0.01f,  
    .kd = 0.049f,
    .err = 0,
    .err_last = 0,
    .integral = 0,
    .dt = 0.01f    
};

extern int8 duty;

/* 积分限幅：防止“积分饱和” */
const float INTEGRAL_MAX = 200.0f;   // 对应 PWM 占空差值
//定义舵机打角值
float servo_motor_angle = SERVO_MOTOR_M; 
//PID系数kp
extern float adc_error;

int8_t duty_straight = 25;
int8_t duty_circle = 20;
float kp_straight = 0.5f;
float kp_circle = 1.7f;

//误差比例系数
float p=9.0f;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机PWM
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     舵机打角值在中值基础上加上电感误差值乘以kp系数得到
//				打角值做限幅处理后直接输出舵机执行，根据实际接线情况选择三个舵机中的一个舵机即可
//-------------------------------------------------------------------------------------------------------------------
/*void set_servo_pwm()
{
	//根据偏差计算舵机打角
	servo_motor_angle = SERVO_MOTOR_M - kp * adc_error;
	//打角限幅
	if(servo_motor_angle > SERVO_MOTOR_R_MAX)servo_motor_angle = SERVO_MOTOR_R_MAX;
	if(servo_motor_angle < SERVO_MOTOR_L_MAX)servo_motor_angle = SERVO_MOTOR_L_MAX;

	//设置舵机打角
	pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}
*/
void set_servo_pwm(void)
{
    if(-0.5<adc_error&&adc_error<0.5){
        servo.kp = kp_straight;
        duty= duty_straight;
    }
    else{
        servo.kp = kp_circle;
        duty =duty_circle;
    }
    
    if(reset==false){
        servo.ki=0.0f;
    }
    else{
         servo.ki=0.01f;
    }
    /* 读误差 */
    servo.err = -adc_error*p;

    /* 积分项 + 限幅 */
    servo.integral += servo.err * servo.dt;
    if (servo.integral  >  INTEGRAL_MAX) servo.integral =  INTEGRAL_MAX;
    if (servo.integral < -INTEGRAL_MAX) servo.integral = -INTEGRAL_MAX;

    /* 微分项 */
    float derivative = (servo.err - servo.err_last) / servo.dt;
    servo.err_last = servo.err;

    /* 位置式 PID 输出 */
    float pid_out = servo.kp * servo.err
                  + servo.ki * servo.integral
                  + servo.kd * derivative;

    /* 把 pid_out 映射到舵机角度 */
    servo_motor_angle =SERVO_MOTOR_M - pid_out - 0.1;
    servo_motor_angle=servo_motor_angle*1;

    /* 限幅 */
    if (servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if (servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;
  
    /* 舵机转向 */
    uint32 duty = (uint32)SERVO_MOTOR_DUTY(servo_motor_angle);
    pwm_set_duty(SERVO_MOTOR1_PWM, duty);
    
}