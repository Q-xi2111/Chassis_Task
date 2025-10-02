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
/*
  电机参数：
        0	左前轮	FL
        1	右前轮	FR  
        2	左后轮	BL
        3	右后轮	BR
*/
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

/*----配置掩码，注意这个函数需要在CANInit之前配置*/
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t object_Para, uint32_t ID, uint32_t Mask_ID);   

void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);    //发送标准数据帧

/*----控制底盘的4个电机，里面包含的是3508电机*/
void CAN_cmd_chassis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);      

/*----初始化can总线----*/
void CANInit(CAN_HandleTypeDef *hcan);

/*----传递电机参数----*/
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
