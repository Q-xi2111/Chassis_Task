#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "chassis_task_assign.h"
#include "Remote_control.h"

/* ---- 第一种模式：停止状态，防止电机发疯 ---- */
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

/* ---- 第二种模式，遥控器控制 ---- */
static void chassis_no_follow_yaw_control(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (v_Forward_set == NULL || v_Move_set == NULL || w_Rotate_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    //计算平动速度，旋转速度
    chassis_rc_to_control_vector(v_Forward_set, v_Move_set,w_Rotate_set, chassis_move_rc_to_vector);
}

/* ---- 第三种模式。调试电机 ---- */
static void chassis_open_set_control(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (v_Forward_set == NULL || v_Move_set == NULL || w_Rotate_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    
    *v_Forward_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Forward_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    //乘以负值，向左为正方向
    *v_Move_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Move_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    /* ---- 向左是逆时针旋转 按键向左打的时候就是逆时针旋转 ---- */
    *w_Rotate_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Rotate_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

//定义为全局变量，初始化为不移动
chassis_behaviour_e  chassis_behaviour_mode  =CHASSIS_NO_MOVE;

/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode){
    if (chassis_move_mode == NULL){
        return;
    }
    /* ---- 中间为静止状态 ---- */
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])){
        chassis_behaviour_mode = CHASSIS_NO_MOVE;                          
    }
    /* ---- 往下打为调试模式 ---- */
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])){
        chassis_behaviour_mode = CHASSIS_OPEN ;                               //静止状态
    }
    /* ---- 往上打为遥控器控制模式 ----*/
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])){
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }

    /* ---- 定义上层模式 ---- */
    if (chassis_behaviour_mode == CHASSIS_NO_MOVE){
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;             //无角度闭环      
    }else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW){
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;             //此时随便转，受到遥控器控制
    }else if (chassis_behaviour_mode == CHASSIS_OPEN){
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}

/* ---- 由遥控器控制上层模式并且给出目标速度 ，这里传递进出的v_Forward_Set,v_Move_set,w_Rotate_set 后续拿出来赋值给控制结构----*/
void chassis_behaviour_control_set(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *w_Rotate_set, 
            chassis_move_t *chassis_move_rc_to_vector){
    if (v_Forward_set == NULL || v_Move_set == NULL || w_Rotate_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    /*----保持速度为0----*/
    if ( chassis_behaviour_mode == CHASSIS_NO_MOVE){
        chassis_no_move_control(v_Forward_set, v_Move_set, w_Rotate_set, chassis_move_rc_to_vector);
    }
    /*----遥控器直接给定目标旋转速度----*/
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW){
        chassis_no_follow_yaw_control(v_Forward_set, v_Move_set, w_Rotate_set, chassis_move_rc_to_vector);
    }
    /*----开环模式，直接将遥控器通道值映射到输出上面来----*/
    else if (chassis_behaviour_mode == CHASSIS_OPEN){
        chassis_open_set_control(v_Forward_set, v_Move_set, w_Rotate_set, chassis_move_rc_to_vector);
    }
}
