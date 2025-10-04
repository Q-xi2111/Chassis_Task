#include "Remote_control.h"


extern DMA_HandleTypeDef hdma_usart3_rx;

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RC_ctrl_t rc_ctrl;


/**
 * @brief   �� SBUS ����֡����Ϊң�����ṹ��
 * @param   sbus_buf  ָ�� DMA �յ��� 18 �ֽ� SBUS ����֡��ֻ������ֹ��д��
 * @param   rc_ctrl   ������������Ŀ��ṹ��
 * @note    ֡��ʽ��Futaba SBUS�������ƽ�������� 100 k��8E2��
 *          �������ٶ��ϲ������ֽ�ȡ����0x00?0xFF�����ֽڶ���
 *          ������С�˷�ʽƴ�ϣ�11 bit/ͨ������ 16 ͨ�� + 2 bit ״̬��־
 */


void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
/*  λ��ժҪ��LSB 0 �𲽣�
 *  ch0   0-10
 *  ch1  11-21
 *  ch2  22-32
 *  ch3  33-43
 *  S1   44-45
 *  S2   46-47
 *  mouseX  48-63
 *  mouseY  64-79
 *  mouseZ  80-95
 *  mouseL  96-103
 *  mouseR 104-111
 *  key    112-127
 */
    if (sbus_buf == NULL || rc_ctrl == NULL) return;

    /*----------- 128 bit ԭʼ����ƴ�� 4 �� -----------*/
    uint32_t w[4];
    w[0] = (uint32_t)sbus_buf[0] <<  0 | (uint32_t)sbus_buf[1] <<  8 |
           (uint32_t)sbus_buf[2] << 16 | (uint32_t)sbus_buf[3] << 24;
    w[1] = (uint32_t)sbus_buf[4] <<  0 | (uint32_t)sbus_buf[5] <<  8 |
           (uint32_t)sbus_buf[6] << 16 | (uint32_t)sbus_buf[7] << 24;
    w[2] = (uint32_t)sbus_buf[8] <<  0 | (uint32_t)sbus_buf[9] <<  8 |
           (uint32_t)sbus_buf[10]<< 16 | (uint32_t)sbus_buf[11]<< 24;
    w[3] = (uint32_t)sbus_buf[12]<<  0 | (uint32_t)sbus_buf[13]<<  8 |
           (uint32_t)sbus_buf[14]<< 16 | (uint32_t)sbus_buf[15]<< 24;

    /*----------- ң��ͨ�� 11bit ��4 --------------------*/
    rc_ctrl->rc.ch[0] = (w[0] >>  0) & 0x07FF;
    rc_ctrl->rc.ch[1] = (w[0] >> 11) & 0x07FF;
    rc_ctrl->rc.ch[2] = ((w[0] >> 22) | (w[1] << 10)) & 0x07FF;
    rc_ctrl->rc.ch[3] = (w[1] >>  1) & 0x07FF;

    /*----------- ���� 2bit ��2 -------------------------*/
    uint32_t sw = (w[1] >> 12) & 0x0F;
    rc_ctrl->rc.s[0] =  sw       & 0x03;   // S1
    rc_ctrl->rc.s[1] = (sw >> 2) & 0x03;   // S2

    /*----------- ����� 16bit �з��� ------------------*/
    rc_ctrl->mouse.x = (int16_t)(w[1] >> 16);                 // 48-63
    rc_ctrl->mouse.y = (int16_t)((w[2] <<  0) >> 16);         // 64-79
    rc_ctrl->mouse.z = (int16_t)((w[2] >> 16) | (w[3] << 16));// 80-95

    /*----------- ��갴�� ----------------------------*/
    rc_ctrl->mouse.press_l = (sbus_buf[12] & 0x01);   // bit96
    rc_ctrl->mouse.press_r = (sbus_buf[13] & 0x01);   // bit104

    /*----------- ���� 16bit --------------------------*/
    rc_ctrl->key.v = (uint16_t)sbus_buf[14] | ((uint16_t)sbus_buf[15] << 8);

    /*----------- ���ƫ�� ----------------------------*/
    for (int i = 0; i < 4; ++i) rc_ctrl->rc.ch[i] -= RC_CH_VALUE_OFFSET;
}

/*
    *@���ܴ���3���ճ�ʼ��
    *@1.ʵ�ֵĹ��ܣ�
        *@1.�� USART3 �� DMA ����բ�ţ�DMAR��
        *@2.�򿪿����жϣ�IDLE��
        *@3.ͣ DMA �� ��Ĵ�����PAR��M0AR/M1AR��NDTR��DBM���� �ٿ� DMA
    *@2.����Ĵ���ֵ��    
        *@PAR�Ĵ���ָ��DMA���˵�����Դ����Զָ��PAR��
        *@MOAR,M1ARָ�����������յ�ַ������DMA�İ�����MOAR,M1AR֮�������л�
        *@NDTRָ����ת���Ĵ���
        *@DBM������1���DMA���������յ�ַ֮�������л�
    *@3.rx_buf_1,rx_buf_2Ϊ�������ĵ�ַ��dam_buf_numberΪת�˵�����
*/
void RC_Init(uint8_t * rx_buf_1,uint8_t * rx_buf_2,uint16_t dam_buf_number){
    SET_BIT(huart3.Instance->CR3,USART_CR3_DMAR);                   //�򿪴���DMA����
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);                     //�򿪴���3�Ŀ����ж�    

    //�ر�DMA,������Ҫ����DMA��ز���
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR& DMA_SxCR_EN){
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    //�Ĵ�����λΪ32
    hdma_usart3_rx.Instance->PAR=(uint32_t)&(USART3->DR);           //�趨���˼Ĵ����ĵ�ַ  ��&���Ĵ����ĵ�ַȡ����
    hdma_usart3_rx.Instance->M0AR=(uint32_t)rx_buf_1;               //�趨ת����  
    hdma_usart3_rx.Instance->M1AR=(uint32_t)rx_buf_2;             
    hdma_usart3_rx.Instance->NDTR=(uint32_t)dam_buf_number;         //����ת������

    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);             //����˫����ģʽ

    __HAL_DMA_ENABLE(&hdma_usart3_rx);                               //ʹ��DMA  
   
    /*----�����жϣ��������޷����뵽�����ж���ȥ��������----*/
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * @brief  USART3 ȫ���жϷ�����
 *         ����**�����ж� + DMA ˫����**��ʽ���� SBUS ����֡
 * @note   1. ������ɺ󴥷� UART_FLAG_IDLE
 *         2. ���� DMA_SxCR_CT �жϵ�ǰʹ�õ��� M0AR ���� M1AR
 *         3. ����ʵ�ʽ��ճ��� �� �������� �� �л������� �� ���� DMA
 * ֮����Ҫ̫���ж��ˣ���sbus_to_rc��ʵ���˶����ݵĴ���
 */
void USART3_IRQHandler(void)
{
    //��ʱ���յ�һ���ֽڣ������д���
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);   // ������д����־��RXNE
    }

    //֮ǰ��ʼ�����˿����жϣ���������յ��ֽھ��Ǳ�ʾһ֡���ݷ������
    else if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))          //ȷ��IDLE����Ϊ��1
    {
        static uint16_t this_time_rx_len = 0;                       // ���ν��ճ���

        __HAL_UART_CLEAR_PEFLAG(&huart3);                           //�崦IDLE��־
        //�������жϻ�������ע��DMA_SxCR_CT�ǳ�������CR��ӦΪ0��RESET��ʱ��ʹ�õ���MOAR
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //�ر� DMA��׼���������� 
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //����ʵ������,��������С-ʵ��ʣ������
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //����װ�ؼ�����
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

    
            //����DMA����ʼ�� Memory 1 д���� 
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            //������ϣ�����д���
            if (this_time_rx_len == RC_FRAME_LENGTH)
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);   // ���� Memory 0 ������
        }

        /* 2.2 ��ǰĿ�껺������ Memory 1��CT = 1�� */
        else        
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /* �л����������� CT �� 0���´� DMA ʹ�� Memory 0 */
            // hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);   // ���� Memory 1 ������
        }
    }
}

/*----�����Ѿ������ң�����շ����ݵĳ�������ֻ��Ҫ��һ���ӿڷ���RC_ctrl_t rc_ctrl;��ʹ�������ļ�����ʹ��*/

/*----����ң����ָ��----*/
const RC_ctrl_t *get_remote_control_point(void){
    return &rc_ctrl;
}
