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
/*
  ���������
        0	��ǰ��	FL
        1	��ǰ��	FR  
        2	�����	BL
        3	�Һ���	BR
*/
#define M3058_ID  0x200      //����ı�ʶ,һ��ֻ��4��
#define GM6020_ID 0x1FF      

/*----ת��ת�٣������=19��1*/
#define M3058_Moderating ratio 19


#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

// �˲�����ţ�����CAN_FILTER(5),�����ֵ��5<<3
#define CAN_FILTER(x) ((x) << 3)

// ���ն���ѡ��
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// ��׼֡����չ֡
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// ����֡��ң��֡
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

typedef struct{
    int16_t angle_speed;                //��¼�����ת�٣�rpm/min
    uint16_t angle;                     //��¼���ת���ĽǶȣ�ע�ⷶΧΪ0~8191,��Ӧ360��
    int16_t electric_current;           //��¼�����Ĵ�С����λ��mA
    uint8_t Temp;                       //��¼�¶ȣ���λ�����϶�
    int16_t last_angle;
}motor_measure_t;

/*----�������룬ע�����������Ҫ��CANInit֮ǰ����*/
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t object_Para, uint32_t ID, uint32_t Mask_ID);   

void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);    //���ͱ�׼����֡

/*----���Ƶ��̵�4������������������3508���*/
void CAN_cmd_chassis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);      

/*----��ʼ��can����----*/
void CANInit(CAN_HandleTypeDef *hcan);

/*----���ݵ������----*/
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

void CAN_Receive_M3508_4Motor(CAN_HandleTypeDef *hcan);


#endif
