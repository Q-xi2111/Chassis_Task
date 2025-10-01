#ifndef __SERIAL_TEST_H__
#define __SERIAL_TEST_H__

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "stm32f4xx_hal_dma.h"
#include "Remote_control.h"

/*----允许DMA发送----*/
void Serial_Start(void);

/*----通过串口一发送数据给PC端----*/
void usart_printf(const char *str,...);

/*----遥控器数据接口----*/
void print_rc_ctrl(const RC_ctrl_t *rc);

#endif
