#ifndef __CHASSIS_TASK_ASSIGN_H__
#define __CHASSIS_TASK_ASSIGN_H__

#include "chassis_task.h"


/*---- ���ڳ�ʼ�����ƽṹ������Ĳ�������Ҫ��ָ���ʼ�� ----*/
void chassis_init(chassis_move_t *chassis_move_init);

void chassis_feedback_update(chassis_move_t *chassis_move_update);

void chassis_set_contorl(chassis_move_t *chassis_move_control);

void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

void chassis_set_mode(chassis_move_t *chassis_move_mode);

#endif

