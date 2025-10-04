#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "chassis_task_assign.h"
#include "Remote_control.h"

/* ---- ��һ��ģʽ��ֹͣ״̬����ֹ������� ---- */
static void chassis_no_move_control(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (v_Forward_set == NULL || v_Move_set == NULL || w_Rotate_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *v_Forward_set = 0.0f;
    *v_Move_set = 0.0f;
    *w_Rotate_set = 0.0f;                         
}

/* ---- �ڶ���ģʽ��ң�������� ---- */
static void chassis_no_follow_yaw_control(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (v_Forward_set == NULL || v_Move_set == NULL || w_Rotate_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    //����ƽ���ٶȣ���ת�ٶ�
    chassis_rc_to_control_vector(v_Forward_set, v_Move_set,w_Rotate_set, chassis_move_rc_to_vector);
}

/* ---- ������ģʽ�����Ե�� ---- */
static void chassis_open_set_control(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (v_Forward_set == NULL || v_Move_set == NULL || w_Rotate_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    
    *v_Forward_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Forward_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    //���Ը�ֵ������Ϊ������
    *v_Move_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Move_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    /* ---- ��������ʱ����ת ����������ʱ�������ʱ����ת ---- */
    *w_Rotate_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Rotate_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

//����Ϊȫ�ֱ�������ʼ��Ϊ���ƶ�
chassis_behaviour_e  chassis_behaviour_mode  =CHASSIS_NO_MOVE;

/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode){
    if (chassis_move_mode == NULL){
        return;
    }
    /* ---- �м�Ϊ��ֹ״̬ ---- */
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])){
        chassis_behaviour_mode = CHASSIS_NO_MOVE;                          
    }
    /* ---- ���´�Ϊ����ģʽ ---- */
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])){
        chassis_behaviour_mode = CHASSIS_OPEN ;                               //��ֹ״̬
    }
    /* ---- ���ϴ�Ϊң��������ģʽ ----*/
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])){
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }

    /* ---- �����ϲ�ģʽ ---- */
    if (chassis_behaviour_mode == CHASSIS_NO_MOVE){
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;             //�޽Ƕȱջ�      
    }else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW){
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;             //��ʱ���ת���ܵ�ң��������
    }else if (chassis_behaviour_mode == CHASSIS_OPEN){
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}

/* ---- ��ң���������ϲ�ģʽ���Ҹ���Ŀ���ٶ� �����ﴫ�ݽ�����v_Forward_Set,v_Move_set,w_Rotate_set �����ó�����ֵ�����ƽṹ----*/
void chassis_behaviour_control_set(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, 
            chassis_move_t *chassis_move_rc_to_vector){
    if (v_Forward_set == NULL || v_Move_set == NULL || w_Rotate_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    /*----�����ٶ�Ϊ0----*/
    if ( chassis_behaviour_mode == CHASSIS_NO_MOVE){
        chassis_no_move_control(v_Forward_set, v_Move_set, w_Rotate_set, chassis_move_rc_to_vector);
    }
    /*----ң����ֱ�Ӹ���Ŀ����ת�ٶ�----*/
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW){
        chassis_no_follow_yaw_control(v_Forward_set, v_Move_set, w_Rotate_set, chassis_move_rc_to_vector);
    }
    /*----����ģʽ��ֱ�ӽ�ң����ͨ��ֵӳ�䵽���������----*/
    else if (chassis_behaviour_mode == CHASSIS_OPEN){
        chassis_open_set_control(v_Forward_set, v_Move_set, w_Rotate_set, chassis_move_rc_to_vector);
    }
}
