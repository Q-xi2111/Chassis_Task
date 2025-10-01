#include "Serial_test.h"

void Serial_Start(void){
    /*----����DMA����----*/
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);   
}

void usart_printf(const char *str,...){
    static uint8_t max_tx_buf[128];
    va_list ap;
    uint16_t length;
    va_start(ap,str);                           //ap����ָ���ַ���������ָ��str����Ŀɱ������Ҫ��⺯����һ��ջ��
    //str�������λ��ap�Ჹȫ��Щ���λ��Ȼ�����õ��ַ�����max_tx_buf��Ȼ��vsprintf�ķ���ֵλmax_tx_buf�ַ����ĳ���
    length=vsprintf((char*)max_tx_buf,str,ap);        
    va_end(ap);
    //��max_tx__buf�а������ݽ��з���
    HAL_UART_Transmit_DMA(&huart1,max_tx_buf,length);
}

void print_rc_ctrl(const RC_ctrl_t *rc){
    usart_printf(
        "ch0:%d\r\n"
        "ch1:%d\r\n"
        "ch2:%d\r\n"
        "ch3:%d\r\n"
        "s1:%d\r\n"
        "s2:%d\r\n"
        "mouse_x:%d\r\n"
        "mouse_y:%d\r\n"
        "mouse_z:%d\r\n"
        "press_l:%d\r\n"
        "press_r:%d\r\n"
        "key:%d\r\n",
        rc->rc.ch[0],
        rc->rc.ch[1],
        rc->rc.ch[2],
        rc->rc.ch[3],
        rc->rc.s[0],
        rc->rc.s[1],
        rc->mouse.x,
        rc->mouse.y,
        rc->mouse.z,
        rc->mouse.press_l,
        rc->mouse.press_r,
        rc->key.v);
}

