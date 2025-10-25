#include "zf_common_headfile.h"
#include "my_common.h"
/* PID �ṹ�� */
struct pid_t{
    float kp, ki, kd;
    float err;       // �������
    float err_last;  // �ϴ����
    float integral;  // �����ۼ�
    float dt;        // �������ڣ���λ s
} ;

/* ��� PID ʵ�� */
struct pid_t servo = {
    .kp = 0.14f,
    .ki = 0.005f,  
    .kd = 0.003f,
    .err = 0,
    .err_last = 0,
    .integral = 0,
    .dt = 0.01f    
};

/* �����޷�����ֹ�����ֱ��͡� */
const float INTEGRAL_MAX = 200.0f;   // ��Ӧ PWM ռ�ղ�ֵ
//���������ֵ
float servo_motor_angle = SERVO_MOTOR_M; 
//PIDϵ��kp
extern float adc_error;

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ö��PWM
// ����˵��     ��
// ���ز���     ��
// ʹ��ʾ��     ��
// ��ע��Ϣ     ������ֵ����ֵ�����ϼ��ϵ�����ֵ����kpϵ���õ�
//				���ֵ���޷������ֱ��������ִ�У�����ʵ�ʽ������ѡ����������е�һ���������
//-------------------------------------------------------------------------------------------------------------------
/*void set_servo_pwm()
{
	//����ƫ����������
	servo_motor_angle = SERVO_MOTOR_M - kp * adc_error;
	//����޷�
	if(servo_motor_angle > SERVO_MOTOR_R_MAX)servo_motor_angle = SERVO_MOTOR_R_MAX;
	if(servo_motor_angle < SERVO_MOTOR_L_MAX)servo_motor_angle = SERVO_MOTOR_L_MAX;

	//���ö�����
	pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}
*/
void set_servo_pwm(void)
{
    /* ����� */
    servo.err = adc_error;

    /* ������ + �޷� */
    servo.integral += servo.err * servo.dt;
    if (servo.integral  >  INTEGRAL_MAX) servo.integral =  INTEGRAL_MAX;
    if (servo.integral < -INTEGRAL_MAX) servo.integral = -INTEGRAL_MAX;

    /* ΢���� */
    float derivative = (servo.err - servo.err_last) / servo.dt;
    servo.err_last = servo.err;

    /* λ��ʽ PID ��� */
    float pid_out = servo.kp * servo.err
                  + servo.ki * servo.integral
                  + servo.kd * derivative;

    /* �� pid_out ӳ�䵽����Ƕ� */
    servo_motor_angle =SERVO_MOTOR_M + pid_out;

    /* �޷� */
    if (servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if (servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    /* ���ת�� */
    uint32 duty = (uint32)SERVO_MOTOR_DUTY(servo_motor_angle);
    pwm_set_duty(SERVO_MOTOR1_PWM, duty);
    
}