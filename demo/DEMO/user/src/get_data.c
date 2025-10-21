#include "zf_common_headfile.h"
#include "my_common.h"

//�����ű�������ֵ�ı���
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;

//�������ź�adc��ر���
uint8 channel_index = 0;
adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] = 
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};
uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

//���ݵ����ֵ����õ������ֵ
float adc_error;

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��������ֵ
// ����˵��     ��
// ���ز���     ��
// ʹ��ʾ��     ��
// ��ע��Ϣ     ��������ֵ�ȶ�ȡ������������
//				�˺���һ���Զ�ȡ��4����������ֵ
//-------------------------------------------------------------------------------------------------------------------
void get_encoder()
{
	encoder_data_1 = encoder_get_count(ENCODER_1);                              // ��ȡ����������
    encoder_clear_count(ENCODER_1);                                             // ��ձ���������

    encoder_data_2 = encoder_get_count(ENCODER_2);                              // ��ȡ����������
    encoder_clear_count(ENCODER_2);                                             // ��ձ���������
    
    encoder_data_3 = encoder_get_count(ENCODER_3);                              // ��ȡ����������
    encoder_clear_count(ENCODER_3);                                             // ��ձ���������

    encoder_data_4 = encoder_get_count(ENCODER_4);                              // ��ȡ����������
    
    encoder_clear_count(ENCODER_4);                                             // ��ձ���������
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡADC��ֵ
// ����˵��     ��
// ���ز���     ��
// ʹ��ʾ��     ��
// ��ע��Ϣ     ��ȡ4·ADC�õ���·��вɼ���ֵ
//				�˴����ֵerrorֱ���ɵ�һ·�͵���·���ֱ������õ��������������㷨�����г���
//-------------------------------------------------------------------------------------------------------------------
void get_adc()
{
	  for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
  {
			adc_buffer[channel_index] = adc_convert(channel_list[channel_index]);   //�ɼ����adc�źţ������˲������е��û��Լ�д�˲�����
  }
	adc_error = (float)(adc_buffer[0] - adc_buffer[3] + 21);//�õ���źż���ƫ����ߵ�в�����adcֵ���
}
	
void get_data()
{
	get_encoder();
	get_adc();
	
}