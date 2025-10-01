#ifndef __MOTOR_H__
#define __MOTOR_H__

/*----ģ������----*/
/*
 *1.ʵ�ֵ������
 *2.��ȡ��������ı��ģ������ṩ�ӿ�
*/

#include "main.h"
#include "can.h"
#include "gpio.h"



#define M3058_ID  0x200      //����ı�ʶ,һ��ֻ��4��
#define GM6020_ID 0x1FF      

/*----ת��ת�٣������=19��1*/
#define M3058_Moderating ratio 19


#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef struct{
    int16_t angle_speed;                //��¼�����ת�٣�rpm/min
    uint16_t angle;                     //��¼���ת���ĽǶȣ�ע�ⷶΧΪ0~8191,��Ӧ360��
    int16_t electric_current;           //��¼�����Ĵ�С����λ��mA
    uint8_t Temp;                       //��¼�¶ȣ���λ�����϶�
    int16_t last_angle;
}motor_measure_t;


#endif
