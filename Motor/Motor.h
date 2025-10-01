#ifndef __MOTOR_H__
#define __MOTOR_H__

/*----模块任务----*/
/*
 *1.实现电机控制
 *2.读取电机反馈的报文，并且提供接口
*/

#include "main.h"
#include "can.h"
#include "gpio.h"



#define M3058_ID  0x200      //电机的标识,一般只用4个
#define GM6020_ID 0x1FF      

/*----转置转速：输出端=19：1*/
#define M3058_Moderating ratio 19


#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef struct{
    int16_t angle_speed;                //记录电机的转速，rpm/min
    uint16_t angle;                     //记录电机转过的角度，注意范围为0~8191,对应360°
    int16_t electric_current;           //记录电流的大小，单位：mA
    uint8_t Temp;                       //记录温度，单位：摄氏度
    int16_t last_angle;
}motor_measure_t;


#endif
