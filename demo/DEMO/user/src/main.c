/*********************************************************************************************************************
* 本例程基于RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）开发
* 
* 修改记录
* 日期              作者                备注
* 2025-09-30        wjm            first version
********************************************************************************************************************/

// *************************** 例程硬件连接说明 ***************************
/*
				连接好电机、主板、驱动板
				将电磁前瞻与运放、运放与主板连接好
			（如果有编码器，将编码器安装好并与主板连接）
			
*/
// *************************** 例程测试说明 ***************************
/*
			本例程包含功能：
			1.采集电磁信号并经过ADC传回MCU
			2.电磁信号直接做差计算偏差
			3.计算舵机打角
			4.设置电机pwm使电机旋转
			
			
			注意！注意！注意！注意！
			注意！注意！注意！注意！
			注意！注意！注意！注意！

			1.测试舵机最左，最右，中值，并在mycommon.h中修改，也可以拆下舵盘，先单独将舵机值设置为90.0，后再将舵盘安装上，则舵机对应值与本例程一致
			2.若担心电机旋转方向与与期望方向相反，可调换电机正负两根线在驱动板上的连接位置（注意不是电池！），也可以直接在speed_control.c中修改DIR的电平
			3.将小车正放在电磁线上，电磁线在小车中轴线上，调节运放的值，使得最左边与最右边电感值相等
			4.用电磁信号计算偏差时，最左电感值和最右电感值默认存放在adc_buffer[0]和adc_buffer[3]中，若接线不同请自行修改
			

*/

// **************************** 代码区域 ****************************

#include "zf_common_headfile.h"
#include "my_common.h"
#include "magnet_detect.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern float adc_normal_buffer[ADC_CHANNEL_NUMBER];
extern int16 encoder_data_1;
extern int16 encoder_data_2;
extern float adc_error;
extern int8 duty;
extern void reset_offset();

struct pid_t{
    float kp, ki, kd;
    float err;       // 本次误差
    float err_last;  // 上次误差
    float integral;  // 积分累加
    float dt;        // 调用周期，单位 s
} ;

extern struct pid_t servo;
extern float p;	

int8 pages = 0;
int8 select = 0;

bool first = true;

void Init_All(void);
void showMain(void);


int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    system_delay_ms(300);           //等待主板其他外设上电完成
	Init_All();						//初始化所有外设
	
	pit_ms_init(PIT_CH, 10);//开启10ms中断
	
    while(1)
    {
        // 此处编写需要循环执行的代码
        key_scanner();
        showMain();
    }
}

//此初始化函数共初始化4路电机接口，4路编码器接口，4路电感ADC，以及3路舵机接口
void Init_All(void)
{
	//初始化电机驱动DRV8701
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

	//初始化编码器（正交）
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式
	
	//初始化adc引脚
	adc_init(ADC_CHANNEL1, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL3, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL4, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
		
	//初始化舵机引脚
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
		
	//初始化屏幕
    tft180_set_dir(TFT180_PORTAIT);
	tft180_init();
    
    //关掉这该死的蜂鸣器
    gpio_init(B11,GPO,0,GPO_PUSH_PULL);
    
    //初始化按键
    key_init(10);
   
    //初始化霍尔传感器
    magnetInit();
   
}

extern void get_data();
extern void set_servo_pwm();
extern void set_speed_pwm();

//中断函数，在主函数中初始化为每10ms执行一次
void pit_handler (void)
{
	
	get_data();//获取传感器数据
	set_servo_pwm();//设置舵机打角
	set_speed_pwm();//设置电机速度
}

//主菜单显示
void showMain()
{
    if(key_get_state(KEY_1)==KEY_SHORT_PRESS)
    {
        if(pages==2){
            pages=0;
        }
        else{
            pages++;
        }
        key_clear_state(KEY_1);
        first=true;
    }
    if(pages==0){
        if(first==true){
           tft180_clear();
           tft180_show_string(20, 0, "Basic-Info");
	       tft180_show_string(0, 15, "ADC1:");
	       tft180_show_string(0, 30, "ADC2:");
	       tft180_show_string(0, 45, "ADC3:");
	       tft180_show_string(0, 60, "ADC4:");
	       tft180_show_string(0, 75, "ERROR");
	       tft180_show_string(0, 90, "ENCODER-L:");
           tft180_show_string(0, 105, "ENCODER-R:");
           first=false;
        }
        tft180_show_int(50, 15, adc_buffer[0], 4);
        tft180_show_int(50, 30, adc_buffer[1], 4);
		tft180_show_int(50, 45, adc_buffer[2], 4);
		tft180_show_int(50, 60, adc_buffer[3], 4);
		tft180_show_float(50, 75, adc_error, 4, 1);
		tft180_show_int(80, 90, encoder_data_1, 4);
        tft180_show_int(80, 105, encoder_data_2, 4);
    }
    if(pages==1){
        if(first==true){
            tft180_clear();
            tft180_show_string(20, 0, "ADC-Info");
            tft180_show_string(0, 15, "ADCNOM1:");
	        tft180_show_string(0, 30, "ADCNOM2:");
	        tft180_show_string(0, 45, "ADCNOM3:");
	        tft180_show_string(0, 60, "ADCNOM4:");
            
             first=false;
        }
         tft180_show_float(70, 15, adc_normal_buffer[0], 4,1);
         tft180_show_float(70, 30, adc_normal_buffer[1], 4,1);
		 tft180_show_float(70, 45, adc_normal_buffer[2], 4,1);
		 tft180_show_float(70, 60, adc_normal_buffer[3], 4,1);
        }
     if(pages==2){
         
          if(first==true){
            tft180_clear();
            tft180_show_string(20, 0, "Value-Set");
            tft180_show_string(8, 15, "Reset-offset");
	        tft180_show_string(8, 30, "Duty");
	        tft180_show_string(8, 45, "PID-kp");
	        tft180_show_string(8, 60, "PID-kd");
            tft180_show_string(8, 75, "error-p");
            first=false;
            tft180_show_string(0, 15+select*15, ">");
        }
         tft180_show_int(75, 30, duty, 4);
		 tft180_show_float(75, 45, servo.kp, 4,1);
		 tft180_show_float(75, 60, servo.kd, 4,1);
         tft180_show_float(75, 75, p, 4,1);
        
        //按下第二个键时修改选中项
         if(key_get_state(KEY_2)==KEY_SHORT_PRESS)
         {
             if(select==4)
             {
                 select=0;
             }
             else
             { 
                 select++;
             }
              key_clear_state(KEY_2);
              first=true;
         }
         
         if(key_get_state(KEY_3)==KEY_SHORT_PRESS)
         {
             switch(select)
             {
                 case 0: 
                     reset_offset();
                     break;
                 case 1:
                     duty+=5;
                     break;
                 case 2:
                     servo.kp +=0.5;
                     break;
                 case 3:
                     servo.kd +=0.1;
                     break;
                 case 4:
                     p+=0.1f;
                     break;
             }
              key_clear_state(KEY_3);
         }
         if(key_get_state(KEY_4)==KEY_SHORT_PRESS)
         {
             switch(select)
             {
                 case 0: 
                     reset_offset();
                     break;
                 case 1:
                     duty-=5;
                     break;
                 case 2:
                     servo.kp -=0.5;
                     break;
                 case 3:
                     servo.kd -=0.1;
                     break;
                 case 4:
                     p-=0.1f;
                     break;
             }
              key_clear_state(KEY_4);
         }
   }
}
