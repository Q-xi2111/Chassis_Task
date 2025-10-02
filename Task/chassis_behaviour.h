#ifndef __CHASSIS_BEHAVIOUR_H__
#define __CHASSIS_BEHAVIOUR_H__

#include "chassis_task.h"



/* ---- 
    *@ brief :������Ϊ�����֣�
    *@ 1.:CHASSIS_NO_MOVE -��ֹģʽ����ֹ���̷���
    *@ 2.:CHASSIS_NO_FOLLOW_YAW -ң����������ת
    *@ 3.��CHASSIS_OPEN -����ģʽ��ֱ���ϵ翴��������Լ��������ݶ�Ӧ��� 
---- */
typedef enum
{
  CHASSIS_NO_MOVE,                      //���̱��ֲ���
  CHASSIS_NO_FOLLOW_YAW,                //ң��������
  CHASSIS_OPEN                          //����ģʽ
} chassis_behaviour_e;

/*----����״̬�£�ң������ֵӳ�䵽����ֵ�ı���*/
#define CHASSIS_OPEN_RC_SCALE 10 //��chassis_open ģ���£�ң�������Ըñ������͵�can��

/* ---- �ⲿ�ӿ� ---- */
extern void chassis_behaviour_control_set(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, chassis_move_t *chassis_move_rc_to_vector);
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

#endif

