#include "magnet_detect.h"

//��ʼ��������������Ӧ����
void magnetInit(){
    gpio_init(D4,GPI,0,GPI_PULL_DOWN);
}

//��ȡ��ǰ����״̬
uint8 getMagnet(){
    return gpio_get_level(D4);
}