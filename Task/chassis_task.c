#include "chassis_task.h"        
#include "chassis_task_assign.h" 
#include "FreeRTOS.h"
#include "task.h"


/* ----�������ֵ������֮��output=input,����ֱ��Ϊ0*/
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



//����һ��ȫ�ֱ���,���ڽ���chassis_move
chassis_move_t chassis_move;

void chassis_rc_to_control_vector(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *vz_set,chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || v_Forward_set == NULL || v_Move_set == NULL || vz_set == NULL)
    {
        return;
    }
    
    /* ----v_Forward_channel,v_Move_channalΪ�������������ֵ---- */
    int16_t v_Forward_channel, v_Move_channel,v_Rotate_channel;

    /* ----v_Forward_set_channel,v_Move_set_channel��ͨ��ֵת���ٶ�ֵ������---- */
    fp32 v_Forward_set_channel, v_Move_set_channel,v_Rotate_set_channel;

    //�������ƣ���Ϊң�������ܴ��ڲ��죬��Ҫ��Ӳ�����棬ҡ�����м䣬��ֵ��Ϊ0����ֹû��ң������ʱ��Ҳ�ڶ���vx_channelΪoutput
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Forward_CHANNEL], v_Forward_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Move_CHANNEL], v_Move_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Rotate_CHANNEL], v_Rotate_channel, CHASSIS_RC_DEADLINE);

/*
  ���������
        0	��ǰ��	FL
        1	��ǰ��	FR  
        2	�����	BL
        3	�Һ���	BR
*/
    /* ---- ����Ŀ���ٶ� ---- */
    v_Forward_set_channel = v_Forward_channel * -CHASSIS_Forward_RC_SEN;    
    v_Move_set_channel = v_Move_channel * -CHASSIS_Move_RC_SEN;
    //�ڽṹ���й涨����ʱ�뷽��Ϊ��ֵ
    v_Rotate_set_channel = v_Rotate_channel * -CHASSIS_Rotate_RC_SEN;
        
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ����룬��ֹͻȻͻ�䵼�µ���͵�ת����v_Forward_set_channelΪԭʼֵ
    /* ----����ı����chassis_cmd_slow_set_Forward����һ���ṹ�壬������������ֵ*/
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_Forward, v_Forward_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_Move, v_Move_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_Rotate, v_Rotate_set_channel);

    /* ---- ������ֱ������,�˲������ҲΪ0 ---- */
    if (v_Forward_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_Forward_RC_SEN && v_Forward_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_Forward_RC_SEN){
        chassis_move_rc_to_vector->chassis_cmd_slow_set_Forward.out = 0.0f;
    }

    if (v_Move_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_Move_RC_SEN && v_Move_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_Move_RC_SEN){
        chassis_move_rc_to_vector->chassis_cmd_slow_set_Move.out = 0.0f;
    }
    
    if (fabsf(v_Rotate_set_channel) < CHASSIS_RC_DEADLINE * CHASSIS_Rotate_RC_SEN){
        chassis_move_rc_to_vector->chassis_cmd_slow_set_Rotate.out = 0.0f;
    }

    /* ---- ���Ŀ��ֵ,�趨Ŀ��ֵ ---- */
    *v_Forward_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_Forward.out;
    *v_Move_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_Move.out;
    *vz_set        = chassis_move_rc_to_vector->chassis_cmd_slow_set_Rotate.out;
}


/* -------------- ���岽�� -------------- */
/*
 1.��ң�����Ƿ��ȡ����ֵ�������޷�һ��
 2.��������ڵ��ID
  ���������
        0	��ǰ��	FL
        1	��ǰ��	FR  
        2	�����	BL
        3	�Һ���	BR
 3.�����Ƿ������Ƿ���룬���Ҳ������Ӽ��ԣ�����ң����
*/


//pvParameters����һ�����裬��Ҫ��������rtos�Ĳ���
void chassis_task(void const *pvParameters){
    /* ---- ��һ��������������һ��ʱ�䣬��Ӳ����ɳ�ʼ����������������ɣ�---- */
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    /* ---- �ڶ�������ɳ�ʼ������ ---- */
    chassis_init(&chassis_move);  

    /* ---- ��ɳ�ʼ��֮�������Ŀ��ƻ��� ---- */
    while(1){
        /* ---- �ı�mode ---- */
        chassis_set_mode(&chassis_move);
        //ע�⡪��feedback֮��ʵ�ʵ��Ե�ʱ���޸�
        /* ---- �õ����ݸ��£�����ͨ��������õ�ʵ�ʵ�V_forward,V_Move,V_Rotate ---- */
        chassis_feedback_update(&chassis_move); 

        /* ---- �����ٶȸ���֮�����õ�Ŀ���ٶ� ---- */
        chassis_set_contorl(&chassis_move); 
        
        /* ---- ֮�����pid���� ---- */
        chassis_control_loop(&chassis_move);

        /* ---- ������� ---- */
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
