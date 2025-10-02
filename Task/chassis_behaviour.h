#ifndef __CHASSIS_BEHAVIOUR_H__
#define __CHASSIS_BEHAVIOUR_H__

#include "chassis_task.h"



/* ---- 
    *@ brief :顶层行为的三种，
    *@ 1.:CHASSIS_NO_MOVE -静止模式，防止底盘发疯
    *@ 2.:CHASSIS_NO_FOLLOW_YAW -遥控器控制旋转
    *@ 3.：CHASSIS_OPEN -调试模式，直接上电看电机极性以及各个数据对应情况 
---- */
typedef enum
{
  CHASSIS_NO_MOVE,                      //底盘保持不动
  CHASSIS_NO_FOLLOW_YAW,                //遥控器控制
  CHASSIS_OPEN                          //调试模式
} chassis_behaviour_e;

/*----开环状态下，遥控器的值映射到电流值的比例*/
#define CHASSIS_OPEN_RC_SCALE 10 //在chassis_open 模型下，遥控器乘以该比例发送到can上

/* ---- 外部接口 ---- */
extern void chassis_behaviour_control_set(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, chassis_move_t *chassis_move_rc_to_vector);
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

#endif

