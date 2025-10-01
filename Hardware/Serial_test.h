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

/*----����DMA����----*/
void Serial_Start(void);

/*----ͨ������һ�������ݸ�PC��----*/
void usart_printf(const char *str,...);

/*----ң�������ݽӿ�----*/
void print_rc_ctrl(const RC_ctrl_t *rc);

#endif
