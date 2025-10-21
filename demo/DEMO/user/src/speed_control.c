#include "zf_common_headfile.h"
#include "my_common.h"

//���Ƶ���ٶȣ���ȡ0-100����̫��ɵ��ͣ���ʹ��3S��أ�ռ�ձ�Ӧ�޷�60%��Ĭ��10%
int8 duty = 20;	

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ��PWM
// ����˵��     ��
// ���ز���     ��
// ʹ��ʾ��     ��
// ��ע��Ϣ     DIR���Ƶ������duty����Ϊռ�ձȿ����ٶȣ������ַ�ת�����Խ���Ӧ��DIR����Ϊ��
//				����MOTOR1��Ӧ�ĵ����ת�����Գ��Ե���gpio_set_level(MOTOR1_DIR, GPIO_LOW)��ʹ�õ�������뷽����ת
//-------------------------------------------------------------------------------------------------------------------
void set_speed_pwm()
{
	//ռ�ձ��޷�
	if(duty > MAX_PWM_DUTY)duty = MAX_PWM_DUTY;
	
	gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                   // DIR����ߵ�ƽ
	pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�

	gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                   // DIR����ߵ�ƽ
	pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�

	//gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                   // DIR����ߵ�ƽ
	//pwm_set_duty(MOTOR3_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�

	//gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                   // DIR����ߵ�ƽ
	//pwm_set_duty(MOTOR4_PWM, duty * (PWM_DUTY_MAX / 100));                   // ����ռ�ձ�
}

