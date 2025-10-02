#include "chassis_task.h"        
#include "chassis_task_assign.h" 
#include "FreeRTOS.h"
#include "task.h"


/* ----如果输入值在死区之外output=input,否则直接为0*/
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



//定义一个全局变量,用于接收chassis_move
chassis_move_t chassis_move;

void chassis_rc_to_control_vector(fp32 *v_Forward_set, fp32 *v_Move_set, fp32 *vz_set,chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || v_Forward_set == NULL || v_Move_set == NULL || vz_set == NULL)
    {
        return;
    }
    
    /* ----v_Forward_channel,v_Move_channal为死区处理后的输出值---- */
    int16_t v_Forward_channel, v_Move_channel,v_Rotate_channel;

    /* ----v_Forward_set_channel,v_Move_set_channel是通道值转换速度值的载体---- */
    fp32 v_Forward_set_channel, v_Move_set_channel,v_Rotate_set_channel;

    //死区限制，因为遥控器可能存在差异，主要是硬件方面，摇杆在中间，其值不为0，防止没动遥控器的时候车也在动，vx_channel为output
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Forward_CHANNEL], v_Forward_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Move_CHANNEL], v_Move_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Rotate_CHANNEL], v_Rotate_channel, CHASSIS_RC_DEADLINE);

/*
  电机参数：
        0	左前轮	FL
        1	右前轮	FR  
        2	左后轮	BL
        3	右后轮	BR
*/
    /* ---- 给定目标速度 ---- */
    v_Forward_set_channel = v_Forward_channel * -CHASSIS_Forward_RC_SEN;    
    v_Move_set_channel = v_Move_channel * -CHASSIS_Move_RC_SEN;
    //在结构体中规定了逆时针方向为正值
    v_Rotate_set_channel = v_Rotate_channel * -CHASSIS_Rotate_RC_SEN;
        
    //一阶低通滤波代替斜波作为底盘速度输入，防止突然突变导致电机猛的转动，v_Forward_set_channel为原始值
    /* ----这里改变的是chassis_cmd_slow_set_Forward，是一个结构体，后续用这个输出值*/
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_Forward, v_Forward_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_Move, v_Move_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_Rotate, v_Rotate_set_channel);

    /* ---- 死区内直接清零,滤波器输出也为0 ---- */
    if (v_Forward_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_Forward_RC_SEN && v_Forward_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_Forward_RC_SEN){
        chassis_move_rc_to_vector->chassis_cmd_slow_set_Forward.out = 0.0f;
    }

    if (v_Move_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_Move_RC_SEN && v_Move_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_Move_RC_SEN){
        chassis_move_rc_to_vector->chassis_cmd_slow_set_Move.out = 0.0f;
    }
    
    if (fabsf(v_Rotate_set_channel) < CHASSIS_RC_DEADLINE * CHASSIS_Rotate_RC_SEN){
        chassis_move_rc_to_vector->chassis_cmd_slow_set_Rotate.out = 0.0f;
    }

    /* ---- 输出目标值,设定目标值 ---- */
    *v_Forward_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_Forward.out;
    *v_Move_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_Move.out;
    *vz_set        = chassis_move_rc_to_vector->chassis_cmd_slow_set_Rotate.out;
}


/* -------------- 整体步骤 -------------- */
/*
 1.调遥控器是否读取到了值，并且限幅一致
 2.按下面调节电调ID
  电机参数：
        0	左前轮	FL
        1	右前轮	FR  
        2	左后轮	BL
        3	右后轮	BR
 3.测试是否轮子是否对齐，并且测试轮子极性，忽略遥控器
*/


//pvParameters就是一个摆设，主要接收来自rtos的参数
void chassis_task(void const *pvParameters){
    /* ---- 第一步，先阻塞任务一段时间，给硬件完成初始化（可能是这个理由）---- */
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    /* ---- 第二步，完成初始化函数 ---- */
    chassis_init(&chassis_move);  

    /* ---- 完成初始化之后进入核心控制环节 ---- */
    while(1){
        /* ---- 改变mode ---- */
        chassis_set_mode(&chassis_move);
        //注意——feedback之后实际调试的时候修改
        /* ---- 得到数据更新，这里通过正解算得到实际的V_forward,V_Move,V_Rotate ---- */
        chassis_feedback_update(&chassis_move); 

        /* ---- 进行速度更新之后解算得到目标速度 ---- */
        chassis_set_contorl(&chassis_move); 
        
        /* ---- 之后进行pid控制 ---- */
        chassis_control_loop(&chassis_move);

        /* ---- 驱动电机 ---- */
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
