#include "Motor.h"

uint32_t used_mailbox;      //接收那个邮箱传递数据
/*----定义电机数据数组*/
motor_measure_t motor_data[4];            //存储4个电机的数据


/**
 * @brief  配置CAN的滤波器,这个只是配置了一个32位过滤器在加一个掩码配置
 * @param  hcan        CAN编号（如 &hcan1 或 &hcan2）
 * @param  object_Para 编号 | FIFOx | ID类型 | 帧类型（组合参数）
 * @param  ID          要过滤的ID
 * @param  Mask_ID     屏蔽位（如 0x3FF 或 0x1FFFFFFF，取决于标准帧或扩展帧）
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t object_Para, uint32_t ID, uint32_t Mask_ID){
    CAN_FilterTypeDef can_filter_init_structure;        //关于过滤器的参数
    // 检测关键传参
    assert_param(hcan != NULL);                            //错误处理
    // 数据帧或者遥控帧在fileter配置中不重要，主要是判断扩展帧或标准帧
    if (object_Para & 0x02)    //第二位判断的是标准帧/扩展帧，看到宏定义当为0时，此时为扩展帧
    {
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 << 16;            //<< 3是留出3位给IDE,RTR和一个空置位
        // 掩码后ID的低16bit，object_Para最后一位表示RTR的取值，符合协议，左移1位，后续得到低16位
        can_filter_init_structure.FilterIdLow = ID << 3 | (object_Para & 0x03) << 1;
        // ID掩码值高16bit，掩码的操作位ID的操作一致
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | (object_Para & 0x03) << 1;
    }else{    
        // 掩码后ID的高16bit，ID为11位只要左移5个单位
        can_filter_init_structure.FilterIdHigh = ID << 5;
        // 掩码后ID的低16bit，后16位只要放IDE,RTR，0这三位
        can_filter_init_structure.FilterIdLow = (object_Para & 0x03) << 1;
        // ID掩码值高16bit，后续同理
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = (object_Para & 0x03) << 1;
    }
    // 滤波器序号，0-27，共28个滤波器
    can_filter_init_structure.FilterBank = object_Para >> 3;    //去除滤波器的值
    // 滤波器绑定FIFOx，只能绑定一个
    can_filter_init_structure.FilterFIFOAssignment = (object_Para >> 2) & 0x01;
    // 使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    // 滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    // 从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 *brief:    开启can通信，并且打开接收中断
 *warning:  报文接收配置要在这个函数调用之前，这一点很重要否则无法收到数据
 */
void CANInit(CAN_HandleTypeDef *hcan){
    HAL_CAN_Start(hcan);                                             // 启动CAN外设
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 开启FIFO0接收中断
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // 开启FIFO1接收中断
}

/*
 *@param  hcan的编号
 *@param  ID   数据帧的ID
 *@param  data 要发送的数据
 *@param  Length 发送数据的长度
 *@return:执行发送，并且返回值检验是否传递成功
 */
void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length){
    CAN_TxHeaderTypeDef tx_header; // CAN_TxHeaderTypeDef结构体包含各种帧
    /* 检测关键传参 */
    assert_param(hcan != NULL);
    assert_param(Data != NULL);
    assert_param(Length <= 8); // CAN 一帧最多 8 字节

    tx_header.StdId = ID;          // 标准 ID（11 位）
    tx_header.ExtId = 0;          // 不使用扩展 ID
    tx_header.IDE = CAN_ID_STD;      // 标准帧
    tx_header.RTR = CAN_RTR_DATA; // 数据帧（非遥控帧）
    tx_header.DLC = Length;          // 数据长度

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
 *@中断函数，对电机数据进行解码
 *@param hcan编号
 *@这里以2006电机或者3508电机作为封装对象，提取数据（电调在0~4之间）
 *@电机的反馈报文一般是1KHz
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef rx_header;            //存放帧头数据
    uint8_t rx_data[8];                        //用于接收返回的数据
    //接收队列0里面的数据
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;                         
    
    if (rx_header.IDE == CAN_ID_STD) {      // 标准帧
        uint32_t idx = rx_header.StdId - 0x201;      //处理0x201~0x204电机数据
        if (idx < 4) {                               // 只处理 0x201~0x204
            motor_data[idx].last_angle=motor_data[idx].angle;            
            motor_data[idx].angle =(rx_data[0] << 8) | rx_data[1];
            motor_data[idx].angle_speed =(rx_data[2] << 8) | rx_data[3];
            motor_data[idx].electric_current=(rx_data[4] << 8) | rx_data[5];
            motor_data[idx].Temp =rx_data[6];
        }
    }        
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i){
    return &motor_data[(i & 0x03)];
}
