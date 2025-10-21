#include "zf_common_headfile.h"
#include "my_common.h"

//���������ֵ
float servo_motor_angle = SERVO_MOTOR_M; 
//PIDϵ��kp
float kp = 0.008;
extern float adc_error;

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ö��PWM
// ����˵��     ��
// ���ز���     ��
// ʹ��ʾ��     ��
// ��ע��Ϣ     ������ֵ����ֵ�����ϼ��ϵ�����ֵ����kpϵ���õ�
//				���ֵ���޷������ֱ��������ִ�У�����ʵ�ʽ������ѡ����������е�һ���������
//-------------------------------------------------------------------------------------------------------------------
void set_servo_pwm()
{
	//����ƫ����������
	servo_motor_angle = SERVO_MOTOR_M + kp * adc_error;
	//����޷�
	if(servo_motor_angle > SERVO_MOTOR_R_MAX)servo_motor_angle = SERVO_MOTOR_R_MAX;
	if(servo_motor_angle < SERVO_MOTOR_L_MAX)servo_motor_angle = SERVO_MOTOR_L_MAX;

	//���ö�����
	pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	//pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	//pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}
