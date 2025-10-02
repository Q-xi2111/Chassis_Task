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


#define SBUS_RX_BUF_NUM 36          //重装值
#define RC_FRAME_LENGTH 18          //一帧的数据大小

/*----遥控器通道对应的值----*/
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/*----------------------- 键盘结构体 -----------------------*/
typedef __packed struct 
{
    uint16_t v;         //按键部分
} key_t;
typedef __packed struct 
{
    int16_t x;          //鼠标x轴
    int16_t y;          //鼠标y轴
    int16_t z;          //鼠标z轴
    uint8_t press_l;    //鼠标左键
    uint8_t press_r;    //鼠标右键
} mouse_t;

/*----------------------- 遥控器结构体 -----------------------*/
typedef __packed struct 
{
    int16_t ch[4];     //0~3,4个通道
    uint8_t s[2];       //两个开关，值的范围0~3
} rc_info_t;

/*----------------------- 总控制结构体 -----------------------*/
typedef __packed struct 
{
    rc_info_t rc;       //遥控器部分
    mouse_t   mouse;    //鼠标部分
    key_t     key;      //键盘部分
} RC_ctrl_t;

/*----对应遥控器上遥感的值----*/
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/*----初始化遥控器----*/
extern void RC_Init(uint8_t * rx_buf_1,uint8_t * rx_buf_2,uint16_t dam_buf_number);

const RC_ctrl_t *get_remote_control_point(void);




#endif
