/*********************************************************************************************************************
* �����̻���RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩����
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2025-09-30        wjm            first version
********************************************************************************************************************/

// *************************** ����Ӳ������˵�� ***************************
/*
				���Ӻõ�������塢������
				�����ǰհ���˷š��˷����������Ӻ�
			������б�����������������װ�ò����������ӣ�
			
*/
// *************************** ���̲���˵�� ***************************
/*
			�����̰������ܣ�
			1.�ɼ�����źŲ�����ADC����MCU
			2.����ź�ֱ���������ƫ��
			3.���������
			4.���õ��pwmʹ�����ת
			
			
			ע�⣡ע�⣡ע�⣡ע�⣡
			ע�⣡ע�⣡ע�⣡ע�⣡
			ע�⣡ע�⣡ע�⣡ע�⣡

			1.���Զ���������ң���ֵ������mycommon.h���޸ģ�Ҳ���Բ��¶��̣��ȵ��������ֵ����Ϊ90.0�����ٽ����̰�װ�ϣ�������Ӧֵ�뱾����һ��
			2.�����ĵ����ת�����������������෴���ɵ�������������������������ϵ�����λ�ã�ע�ⲻ�ǵ�أ�����Ҳ����ֱ����speed_control.c���޸�DIR�ĵ�ƽ
			3.��С�������ڵ�����ϣ��������С���������ϣ������˷ŵ�ֵ��ʹ������������ұߵ��ֵ���
			4.�õ���źż���ƫ��ʱ��������ֵ�����ҵ��ֵĬ�ϴ����adc_buffer[0]��adc_buffer[3]�У������߲�ͬ�������޸�
			

*/

// **************************** �������� ****************************

#include "zf_common_headfile.h"
#include "my_common.h"
#include "magnet_detect.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern int16 encoder_data_1;
extern int16 encoder_data_2;
extern float adc_error;
extern int8 duty;	
extern uint16_t adc_MAX[4];
extern int16 offset;
extern void reset_offset();

int8 pages = 0;
bool first = true;

void Init_All(void);
void showMain(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    debug_init();                   // ���Զ˿ڳ�ʼ��

    system_delay_ms(300);           //�ȴ��������������ϵ����
	Init_All();						//��ʼ����������
	
	pit_ms_init(PIT_CH, 10);//����10ms�ж�
	
    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        key_scanner();
        reset_offset();
        showMain();
    }
}

//�˳�ʼ����������ʼ��4·����ӿڣ�4·�������ӿڣ�4·���ADC���Լ�3·����ӿ�
void Init_All(void)
{
	//��ʼ���������DRV8701
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

	//��ʼ����������������
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // ��ʼ��������ģ�������� �������������ģʽ
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // ��ʼ��������ģ�������� �������������ģʽ
	
	//��ʼ��adc����
	adc_init(ADC_CHANNEL1, ADC_12BIT);                                          // ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ����
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          // ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ����
    adc_init(ADC_CHANNEL3, ADC_12BIT);                                          // ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ����
    adc_init(ADC_CHANNEL4, ADC_12BIT);                                          // ��ʼ����Ӧ ADC ͨ��Ϊ��Ӧ����
		
	//��ʼ���������
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
		
	//��ʼ����Ļ
    tft180_set_dir(TFT180_PORTAIT);
	tft180_init();
    
    //�ص�������ķ�����
    gpio_init(B11,GPO,0,GPO_PUSH_PULL);
    
    //��ʼ������
    key_init(10);
   
    //��ʼ������������
    magnetInit();
   
}

extern void get_data();
extern void set_servo_pwm();
extern void set_speed_pwm();

//�жϺ��������������г�ʼ��Ϊÿ10msִ��һ��
void pit_handler (void)
{
	
	get_data();//��ȡ����������
	set_servo_pwm();//���ö�����
	set_speed_pwm();//���õ���ٶ�
}

//���˵���ʾ
void showMain()
{
    if(key_get_state(KEY_2)==KEY_SHORT_PRESS)
    {
        if(pages==1){
            pages=0;
        }
        else{
            pages++;
        }
        key_clear_state(KEY_2);
        first=true;
    }
    if(pages==0){
        if(first==true){
           tft180_clear();
           tft180_show_string(20, 0, "Basic_Info");
	       tft180_show_string(0, 15, "ADC1:");
	       tft180_show_string(0, 30, "ADC2:");
	       tft180_show_string(0, 45, "ADC3:");
	       tft180_show_string(0, 60, "ADC4:");
	       tft180_show_string(0, 75, "ERROR");
	       tft180_show_string(0, 90, "ENCODER-L:");
           tft180_show_string(0, 105, "ENCODER-R:");
           tft180_show_string(0, 120, "DUTY:");
            
           first=false;
        }
        tft180_show_int(50, 15, adc_buffer[0], 4);
        tft180_show_int(50, 30, adc_buffer[1], 4);
		tft180_show_int(50, 45, adc_buffer[2], 4);
		tft180_show_int(50, 60, adc_buffer[3], 4);
		tft180_show_float(50, 75, adc_error, 4, 1);
		tft180_show_int(80, 90, encoder_data_1, 4);
        tft180_show_int(80, 105, encoder_data_2, 4);
		tft180_show_uint(50, 120, duty, 3);
    }
    if(pages==1){
        if(first==true){
            tft180_clear();
            tft180_show_string(20, 0, "ADC_Info");
            tft180_show_string(0, 15, "ADCMAX1:");
	        tft180_show_string(0, 30, "ADCMAX2:");
	        tft180_show_string(0, 45, "ADCMAX3:");
	        tft180_show_string(0, 60, "ADCMAX4:");
            tft180_show_string(0, 75, "offset:");
            
             first=false;
        }
         tft180_show_int(70, 15, adc_MAX[0], 4);
         tft180_show_int(70, 30, adc_MAX[1], 4);
		 tft180_show_int(70, 45, adc_MAX[2], 4);
		 tft180_show_int(70, 60, adc_MAX[3], 4);
		 tft180_show_float(50, 75, offset,4, 1);
    }
}