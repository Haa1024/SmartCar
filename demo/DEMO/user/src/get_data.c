#include "zf_common_headfile.h"
#include "my_common.h"
#include "math.h"

//定义存放编码器数值的变量
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;

//定义电磁信号adc相关变量
uint8 channel_index = 0;
adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] = 
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};

uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

//记录对应通道偏置值
float adc_offset=0.0;

//储存归一化后的电感值
float adc_normal_buffer[ADC_CHANNEL_NUMBER];

//定义差比和差公式中的系数

float coef_A = 2.5;
float coef_B = 3.5;
float coef_C = 2.0;

//能测量到最大的电感值
uint16_t adc_MAX[ADC_CHANNEL_NUMBER]={3726,3725,3100,3793};

//根据电感数值计算得到的误差值
float adc_error;
float adc_error_former;


//-------------------------- 滤波参数优化 --------------------------
// 滑动平均滤波窗口大小（越大滤波效果越强，响应越慢，建议5-10）
#define FILTER_WINDOW_SIZE 8
// 每个通道的历史数据缓冲区（保存最近N次采样值）
uint16_t adc_history[ADC_CHANNEL_NUMBER][FILTER_WINDOW_SIZE] = {0};
// 每个通道的历史数据索引（用于循环更新缓冲区）
uint8 adc_history_index[ADC_CHANNEL_NUMBER] = {0};
// 每个通道的历史数据总和（用于快速计算平均值）
uint32_t adc_history_sum[ADC_CHANNEL_NUMBER] = {0};


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取编码器数值
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     编码器数值先读取，后立即清零
//				此函数一次性读取了4个编码器数值
//-------------------------------------------------------------------------------------------------------------------
void get_encoder()
{
	encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    encoder_data_2 = encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
    
    encoder_data_3 = encoder_get_count(ENCODER_3);                              // 获取编码器计数
    encoder_clear_count(ENCODER_3);                                             // 清空编码器计数

    encoder_data_4 = encoder_get_count(ENCODER_4);                              // 获取编码器计数
    
    encoder_clear_count(ENCODER_4);                                             // 清空编码器计数
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取ADC数值
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     读取4路ADC得到四路电感采集数值
//				此处误差值error直接由第一路和第四路电感直接做差得到，其余误差计算算法可自行尝试
//-------------------------------------------------------------------------------------------------------------------
//void get_adc()
//{
//	  for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
// {
//			adc_buffer[channel_index] = adc_convert(channel_list[channel_index]);   //采集电磁adc信号，若需滤波，自行调用或自己写滤波函数
// }
//	adc_error = (float)(adc_buffer[0] - adc_buffer[3] + offset);//用电磁信号计算偏差，两边电感采样的adc值相减
//}
	
void get_adc()
{
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {   
        // 1. 读取新的ADC采样值
        uint16_t new_adc = adc_convert(channel_list[channel_index]);   
        // 2. 更新历史数据缓冲区：移除最旧的值，加入新值
        // 减去缓冲区中最旧的数据（索引位置的上一个值）
        adc_history_sum[channel_index] -= adc_history[channel_index][adc_history_index[channel_index]];
        // 保存新数据到缓冲区
        adc_history[channel_index][adc_history_index[channel_index]] = new_adc;
        // 加上新数据到总和
        adc_history_sum[channel_index] += new_adc;
        // 3. 更新索引（循环覆盖旧数据）
        adc_history_index[channel_index] = (adc_history_index[channel_index] + 1) % FILTER_WINDOW_SIZE;
        // 4. 计算平均值作为滤波后的值（滑动平均核心）
        adc_buffer[channel_index] = adc_history_sum[channel_index] / FILTER_WINDOW_SIZE;
    }
    
    //计算归一化后的电感值
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
            float upper =(float)adc_buffer[channel_index];
            float lower = (float)adc_MAX[channel_index];
            adc_normal_buffer[channel_index]=upper/lower;
    }
    
    // 计算误差值
    float upper=coef_A*(adc_normal_buffer[0] - adc_normal_buffer[3])+ coef_B*(adc_normal_buffer[1] - adc_normal_buffer[2])+0.0001f-adc_offset;
    float lower=coef_A*(adc_normal_buffer[0] + adc_normal_buffer[3])+fabs(adc_normal_buffer[1] - adc_normal_buffer[2])*coef_C+0.01f;
        
    adc_error_former=adc_error;
    adc_error = upper/lower;
    if(fabs(adc_error-adc_error_former)>0.3f){
        adc_error=(adc_error+adc_error_former)/2.0;
    }
    
}
	




void get_data()
{
	get_encoder();
	get_adc();
	
}

void reset_offset()
{
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
          adc_offset=coef_A*(adc_normal_buffer[0] - adc_normal_buffer[3])+ coef_B*(adc_normal_buffer[1] - adc_normal_buffer[2]);
    }
}
