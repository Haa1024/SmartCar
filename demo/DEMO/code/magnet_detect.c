#include "magnet_detect.h"

uint8 magnet_last = 1;
uint8 times = 0;
//初始化霍尔传感器对应引脚
void magnetInit(){
    gpio_init(D4,GPI,0,GPI_PULL_DOWN);
}

//获取当前引脚状态
bool getMagnet(){
    uint8 magnet = gpio_get_level(D4);
    if(magnet==0&&magnet_last==1){
         times++;
    }
    magnet_last = magnet;
    if(times==2){
        times=0;
    return false;
    }
    else{return true;}
}