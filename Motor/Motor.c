#include "Motor.h"

uint32_t used_mailbox;      //�����Ǹ����䴫������
/*----��������������*/
motor_measure_t motor_data[4];            //�洢4�����������


/**
 * @brief  ����CAN���˲���,���ֻ��������һ��32λ�������ڼ�һ����������
 * @param  hcan        CAN��ţ��� &hcan1 �� &hcan2��
 * @param  object_Para ��� | FIFOx | ID���� | ֡���ͣ���ϲ�����
 * @param  ID          Ҫ���˵�ID
 * @param  Mask_ID     ����λ���� 0x3FF �� 0x1FFFFFFF��ȡ���ڱ�׼֡����չ֡��
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t object_Para, uint32_t ID, uint32_t Mask_ID){
    CAN_FilterTypeDef can_filter_init_structure;        //���ڹ������Ĳ���
    // ���ؼ�����
    assert_param(hcan != NULL);                            //������
    // ����֡����ң��֡��fileter�����в���Ҫ����Ҫ���ж���չ֡���׼֡
    if (object_Para & 0x02)    //�ڶ�λ�жϵ��Ǳ�׼֡/��չ֡�������궨�嵱Ϊ0ʱ����ʱΪ��չ֡
    {
        // �����ID�ĸ�16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 << 16;            //<< 3������3λ��IDE,RTR��һ������λ
        // �����ID�ĵ�16bit��object_Para���һλ��ʾRTR��ȡֵ������Э�飬����1λ�������õ���16λ
        can_filter_init_structure.FilterIdLow = ID << 3 | (object_Para & 0x03) << 1;
        // ID����ֵ��16bit������Ĳ���λID�Ĳ���һ��
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID����ֵ��16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | (object_Para & 0x03) << 1;
    }else{    
        // �����ID�ĸ�16bit��IDΪ11λֻҪ����5����λ
        can_filter_init_structure.FilterIdHigh = ID << 5;
        // �����ID�ĵ�16bit����16λֻҪ��IDE,RTR��0����λ
        can_filter_init_structure.FilterIdLow = (object_Para & 0x03) << 1;
        // ID����ֵ��16bit������ͬ��
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID����ֵ��16bit
        can_filter_init_structure.FilterMaskIdLow = (object_Para & 0x03) << 1;
    }
    // �˲�����ţ�0-27����28���˲���
    can_filter_init_structure.FilterBank = object_Para >> 3;    //ȥ���˲�����ֵ
    // �˲�����FIFOx��ֻ�ܰ�һ��
    can_filter_init_structure.FilterFIFOAssignment = (object_Para >> 2) & 0x01;
    // ʹ���˲���
    can_filter_init_structure.FilterActivation = ENABLE;
    // �˲���ģʽ������ID����ģʽ
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32λ�˲�
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    // �ӻ�ģʽѡ��ʼ��Ԫ
    can_filter_init_structure.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 *brief:    ����canͨ�ţ����Ҵ򿪽����ж�
 *warning:  ���Ľ�������Ҫ�������������֮ǰ����һ�����Ҫ�����޷��յ�����
 */
void CANInit(CAN_HandleTypeDef *hcan){
    HAL_CAN_Start(hcan);                                             // ����CAN����
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // ����FIFO0�����ж�
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // ����FIFO1�����ж�
}

/*
 *@param  hcan�ı��
 *@param  ID   ����֡��ID
 *@param  data Ҫ���͵�����
 *@param  Length �������ݵĳ���
 *@return:ִ�з��ͣ����ҷ���ֵ�����Ƿ񴫵ݳɹ�
 */
void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length){
    CAN_TxHeaderTypeDef tx_header; // CAN_TxHeaderTypeDef�ṹ���������֡
    /* ���ؼ����� */
    assert_param(hcan != NULL);
    assert_param(Data != NULL);
    assert_param(Length <= 8); // CAN һ֡��� 8 �ֽ�

    tx_header.StdId = ID;          // ��׼ ID��11 λ��
    tx_header.ExtId = 0;          // ��ʹ����չ ID
    tx_header.IDE = CAN_ID_STD;      // ��׼֡
    tx_header.RTR = CAN_RTR_DATA; // ����֡����ң��֡��
    tx_header.DLC = Length;          // ���ݳ���

    HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox);
}

void CAN_cmd_chassis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4){
    uint8_t data[8];
    data[0]=motor1 >> 8;
    data[1]=motor1;
    data[2]=motor2 >>8;
    data[3]=motor2;
    data[4]=motor3 >>8;
    data[5]=motor3;
    data[6]=motor4 >>8;
    data[7]=motor4;
    CAN_Send_Data(&CHASSIS_CAN,M3058_ID,data,8);
}

/*
 *@�жϺ������Ե�����ݽ��н���
 *@param hcan���
 *@������2006�������3508�����Ϊ��װ������ȡ���ݣ������0~4֮�䣩
 *@����ķ�������һ����1KHz
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef rx_header;            //���֡ͷ����
    uint8_t rx_data[8];                        //���ڽ��շ��ص�����
    //���ն���0���������
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;                         
    
    if (rx_header.IDE == CAN_ID_STD) {      // ��׼֡
        uint32_t idx = rx_header.StdId - 0x201;      //����0x201~0x204�������
        if (idx < 4) {                               // ֻ���� 0x201~0x204
            motor_data[idx].last_angle=motor_data[idx].angle;            
            motor_data[idx].angle =(rx_data[0] << 8) | rx_data[1];
            motor_data[idx].angle_speed =(rx_data[2] << 8) | rx_data[3];
            motor_data[idx].electric_current=(rx_data[4] << 8) | rx_data[5];
            motor_data[idx].Temp =rx_data[6];
        }
    }        
}

/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i){
    return &motor_data[(i & 0x03)];
}
