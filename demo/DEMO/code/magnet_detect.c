#include "magnet_detect.h"

//初始化霍尔传感器对应引脚
void magnetInit(){
    gpio_init(D4,GPI,0,GPI_PULL_DOWN);
}

//获取当前引脚状态
uint8 getMagnet(){
    return gpio_get_level(D4);
}