#include "chassis_task.h"        // 这里才有 struct 本体
#include "chassis_task_assign.h" // 自己的原型
#include "Motor.h"       // motor_measure_t
#include "FreeRTOS.h"            // 如果用到 vTaskDelay
#include "task.h"
#include "chassis_behaviour.h"
#include "pid.h"

void chassis_init(chassis_move_t *chassis_move_init){
      if (chassis_move_init == NULL)
    {
        return;
    }

    /*---- 计入pid各项参数的值，后续赋值给各个电机pid控制器 ----*/
    const static fp32 motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
  

    /*---- 这里定义平滑时间常数 τ，阶跃响应上升到最终值的 63.2 % 所需的时间，单位ms ----*/
    const static fp32 chassis_Forward_order_filter[1] = {CHASSIS_ACCEL_Forward_NUM};
    const static fp32 chassis_Move_order_filter[1] = {CHASSIS_ACCEL_Move_NUM};
    const static fp32 chassis_Rotate_order_filter[1]={CHASSIS_ACCEL_ROTATE_NUM};
    uint8_t i;

    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();

    /*----获取底盘电机数据指针，初始化PID，这里共用一套pid，是要位置式pid---- */
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        //初始化各个电机pid参数
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    }
    
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_Forward, CHASSIS_CONTROL_TIME, chassis_Forward_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_Move, CHASSIS_CONTROL_TIME, chassis_Move_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_Rotate, CHASSIS_CONTROL_TIME, chassis_Rotate_order_filter);


    //更新最大，最小速度
    chassis_move_init->v_Forward_max_speed=MAX_CHASSIS_Forward_Speed;
    chassis_move_init->v_Move_max_speed=MAX_CHASSIS_Move_Speed;
    chassis_move_init->w_Rotate_max_speed=MAX_CHASSIS_Rotate_Speed;


    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

void chassis_set_mode(chassis_move_t *chassis_move_mode){
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //改变mode
    chassis_behaviour_mode_set(chassis_move_mode);
}



void chassis_feedback_update(chassis_move_t *chassis_move_update){
    if (chassis_move_update == NULL) return;

    /* ---------- 获取电机转速、更新速度，加速度 ---------- */
    for (uint8_t i = 0; i < 4; ++i){
        /*---- 求出实际速度 ，主要是修改这里，得到4个电机的数据 ---- */
        chassis_move_update->motor_chassis[i].speed =
            MOTOR_SPEED_TO_CHASSIS_SPEED_Forward *
            chassis_move_update->motor_chassis[i].chassis_motor_measure->angle_speed;

        /* 加速度 = 速度 PID 微分项 × 控制周期  (m/s?)  */
        chassis_move_update->motor_chassis[i].accel =
            chassis_move_update->motor_speed_pid[i].Dbuf[0] *
            CHASSIS_CONTROL_FREQUENCE;
    }
/*
  电机参数：
        0	左前轮	FL
        1	右前轮	FR  
        2	左后轮	BL
        3	右后轮	BR
*/
/* ---- 这一段正解代码其实无所谓，单纯底盘控制用不到这段代码 ---- */
    const fp32 w0 = chassis_move_update->motor_chassis[0].speed;
    const fp32 w1 = chassis_move_update->motor_chassis[1].speed;
    const fp32 w2 = chassis_move_update->motor_chassis[2].speed;
    const fp32 w3 = chassis_move_update->motor_chassis[3].speed;
   
    chassis_move_update->v_Forward = (-w0 + w1 + w2 - w3);  
    chassis_move_update->v_Move = (-w0 - w1 + w2 + w3);
    chassis_move_update->w_Rotate = (-w0 - w1 - w2 - w3) / MOTOR_DISTANCE_TO_CENTER;
}


void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }
    fp32 v_Forward_set = 0.0f, v_Move_set = 0.0f, w_Rotate_set = 0.0f;
    /*----这里是遥控器控制上层控制的行为模式，依据chassis_behaviour给出v_Forward_set,v_Move_set,w_Rotate_set ----*/
    chassis_behaviour_control_set(&v_Forward_set, &v_Move_set, &w_Rotate_set, chassis_move_control);

    /* ---- 后续根据这些具体的模式来分配，本质就是多进行了一层封装 ----*/

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW){
        //“w_Rotate_set” 是旋转速度控制
            /* 1. 限幅 */
        v_Forward_set = fp32_constrain(v_Forward_set,
                                    -chassis_move_control->v_Forward_max_speed,
                                    chassis_move_control->v_Forward_max_speed);
        v_Move_set    = fp32_constrain(v_Move_set,
                                    -chassis_move_control->v_Move_max_speed,
                                    chassis_move_control->v_Move_max_speed);
        w_Rotate_set  = fp32_constrain(w_Rotate_set,
                                    -chassis_move_control->w_Rotate_max_speed,
                                    chassis_move_control->w_Rotate_max_speed);

        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_Forward, v_Forward_set);
        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_Move,    v_Move_set);
        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_Rotate, w_Rotate_set);

        /* 3. 把滤波结果写回结构体，闭环才能拿到非零值 */
        chassis_move_control->v_Forward_set = chassis_move_control->chassis_cmd_slow_set_Forward.out;
        chassis_move_control->v_Move_set    = chassis_move_control->chassis_cmd_slow_set_Move.out;
        chassis_move_control->w_Rotate_set  = chassis_move_control->chassis_cmd_slow_set_Rotate.out;
    }else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW){
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->v_Forward_set = v_Forward_set;
        chassis_move_control->v_Move_set = v_Move_set;
        chassis_move_control->w_Rotate_set = w_Rotate_set;
        chassis_move_control-> chassis_cmd_slow_set_Forward.out=0.0f;   //使用一阶低通滤波减缓设定值
        chassis_move_control->chassis_cmd_slow_set_Move.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_Rotate.out =0.0f;
    }
}

/*
  电机参数：
        0	左前轮	FL
        1	右前轮	FR  
        2	左后轮	BL
        3	右后轮	BR
*/
/* ---- 运动学逆解算得到各个电机的目标速度 ---- */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{                   
    const fp32 r = MOTOR_DISTANCE_TO_CENTER;            

    wheel_speed[0] = (-vx_set + vy_set + wz_set * r) ;   // FL
    wheel_speed[1] = ( vx_set + vy_set + wz_set * r) ;   // FR
    wheel_speed[2] = ( vx_set - vy_set + wz_set * r) ;   // BL
    wheel_speed[3] = (-vx_set - vy_set + wz_set * r) ;   // BR
}


/* ---- 进行闭环控制*/
void chassis_control_loop(chassis_move_t *chassis_move_control_loop){
    fp32 max_vector = 0.0f;      // 电机速度限幅
    fp32 vector_rate = 0.0f;     // 统一缩放系数，当某个电机速度过大时，同一缩放
    fp32 temp = 0.0f;            // 临时变量
    fp32 wheel_speed[4] = {0};   // 4 个轮子目标线速度 [m/s]
    uint8_t i = 0;

    /* ---- 运动学逆解算得到各电机目标速度 ---- */
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->v_Forward_set,
                                          chassis_move_control_loop->v_Move_set,
                                          chassis_move_control_loop->w_Rotate_set,
                                          wheel_speed);

    /* ---- 调式模式 ---- */
    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0; i < 4; i++)
        {
            /* ----  限速保护 ---- */
            int16_t raw = (int16_t)(wheel_speed[i]);
            raw = fp32_constrain(raw, -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        int16_t rpm = chassis_move_control_loop->motor_chassis[0]
                  .chassis_motor_measure->angle_speed;
        fp32    ms  = chassis_move_control_loop->motor_chassis[0].speed;

            usart_printf("RPM:%d  m/s:%.3f  vx:%.2f  vy:%.2f  wz:%.2f\r\n",
             rpm, ms,
             chassis_move_control_loop->v_Forward_set,
             chassis_move_control_loop->v_Move_set,
             chassis_move_control_loop->w_Rotate_set);
        // usart_printf("%f %f %f %f %f\r\n",chassis_move_control_loop->v_Forward_set,
        //                         chassis_move_control_loop->v_Move_set,
        //                         chassis_move_control_loop->w_Rotate,
        //                         chassis_move_control_loop->motor_chassis[0].speed,
        //                         chassis_move_control_loop->motor_chassis[0].speed_set
        //                     );
        return;                      
    }

    /* ----  限速保护 ---- */
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i]; // 目标速度
        //取电机线速度最大值
        temp = fabsf(wheel_speed[i]);
        if (temp > max_vector) max_vector = temp;                              // 找出最大max_vector
    }
    //如果有轮子的速度大于限幅，则统一缩放
    if (max_vector > MAX_WHEEL_SPEED)            
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector; // 统一缩放系数，这个系数对底盘达到目标速度影响最小
        for (i = 0; i < 4; i++)
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
    /* ---- pid控制得到输出值 ---- */
    for (i = 0; i < 4; i++)
    {
        //传入的参数是m/s，数量级是个位数，输出值对应速度4.2*10（**-4）,
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
                 chassis_move_control_loop->motor_chassis[i].speed,      // 反馈（m/s）
                 chassis_move_control_loop->motor_chassis[i].speed_set); // 目标（m/s）
    }
    //这里进行打印，5个参数，从左到右依次是 vx_set,vy_set,vw_set,轮子1的实际速度，轮子1的目标速度
    int16_t rpm = chassis_move_control_loop->motor_chassis[0]
                  .chassis_motor_measure->angle_speed;
    fp32    ms  = chassis_move_control_loop->motor_chassis[0].speed;

    
    usart_printf("RPM:%d  m/s:%.3f  vx:%.2f  vy:%.2f  wz:%.2f\r\n",
             rpm, ms,
             chassis_move_control_loop->v_Forward_set,
             chassis_move_control_loop->v_Move_set,
             chassis_move_control_loop->w_Rotate_set);
    //  usart_printf("%f %f %f %f %f\r\n",chassis_move_control_loop->v_Forward_set,
    //                                chassis_move_control_loop->v_Move_set,
    //                                chassis_move_control_loop->w_Rotate,
    //                                chassis_move_control_loop->motor_chassis[0].speed,
    //                                chassis_move_control_loop->motor_chassis[0].speed_set
    //                              );
    /* ---- 发出数据---- */
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current =
            (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
