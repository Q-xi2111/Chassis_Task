#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__


#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "stm32f4xx_hal_dma.h"


#define SBUS_RX_BUF_NUM 36          //��װֵ
#define RC_FRAME_LENGTH 18          //һ֡�����ݴ�С

/*----ң����ͨ����Ӧ��ֵ----*/
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/*----------------------- ���̽ṹ�� -----------------------*/
typedef __packed struct 
{
    uint16_t v;         //��������
} key_t;
typedef __packed struct 
{
    int16_t x;          //���x��
    int16_t y;          //���y��
    int16_t z;          //���z��
    uint8_t press_l;    //������
    uint8_t press_r;    //����Ҽ�
} mouse_t;

/*----------------------- ң�����ṹ�� -----------------------*/
typedef __packed struct 
{
    int16_t ch[4];     //0~3,4��ͨ��
    uint8_t s[2];       //�������أ�ֵ�ķ�Χ0~3
} rc_info_t;

/*----------------------- �ܿ��ƽṹ�� -----------------------*/
typedef __packed struct 
{
    rc_info_t rc;       //ң��������
    mouse_t   mouse;    //��겿��
    key_t     key;      //���̲���
} RC_ctrl_t;

/*----��Ӧң������ң�е�ֵ----*/
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/*----��ʼ��ң����----*/
extern void RC_Init(uint8_t * rx_buf_1,uint8_t * rx_buf_2,uint16_t dam_buf_number);

const RC_ctrl_t *get_remote_control_point(void);




#endif
