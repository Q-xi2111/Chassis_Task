#include "chassis_task.h"        // ������� struct ����
#include "chassis_task_assign.h" // �Լ���ԭ��
#include "Motor.h"       // motor_measure_t
#include "FreeRTOS.h"            // ����õ� vTaskDelay
#include "task.h"
#include "chassis_behaviour.h"
#include "pid.h"

void chassis_init(chassis_move_t *chassis_move_init){
      if (chassis_move_init == NULL)
    {
        return;
    }

    /*---- ����pid���������ֵ��������ֵ���������pid������ ----*/
    const static fp32 motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
  

    /*---- ���ﶨ��ƽ��ʱ�䳣�� �ӣ���Ծ��Ӧ����������ֵ�� 63.2 % �����ʱ�䣬��λms ----*/
    const static fp32 chassis_Forward_order_filter[1] = {CHASSIS_ACCEL_Forward_NUM};
    const static fp32 chassis_Move_order_filter[1] = {CHASSIS_ACCEL_Move_NUM};
    const static fp32 chassis_Rotate_order_filter[1]={CHASSIS_ACCEL_ROTATE_NUM};
    uint8_t i;

    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();

    /*----��ȡ���̵������ָ�룬��ʼ��PID�����ﹲ��һ��pid����Ҫλ��ʽpid---- */
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        //��ʼ���������pid����
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    }
    
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_Forward, CHASSIS_CONTROL_TIME, chassis_Forward_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_Move, CHASSIS_CONTROL_TIME, chassis_Move_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_Rotate, CHASSIS_CONTROL_TIME, chassis_Rotate_order_filter);


    //���������С�ٶ�
    chassis_move_init->v_Forward_max_speed=MAX_CHASSIS_Forward_Speed;
    chassis_move_init->v_Move_max_speed=MAX_CHASSIS_Move_Speed;
    chassis_move_init->w_Rotate_max_speed=MAX_CHASSIS_Rotate_Speed;


    //����һ������
    chassis_feedback_update(chassis_move_init);
}

void chassis_set_mode(chassis_move_t *chassis_move_mode){
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //�ı�mode
    chassis_behaviour_mode_set(chassis_move_mode);
}



void chassis_feedback_update(chassis_move_t *chassis_move_update){
    if (chassis_move_update == NULL) return;

    /* ---------- ��ȡ���ת�١������ٶȣ����ٶ� ---------- */
    for (uint8_t i = 0; i < 4; ++i){
        /*---- ���ʵ���ٶ� ����Ҫ���޸�����õ�4����������� ---- */
        chassis_move_update->motor_chassis[i].speed =
            MOTOR_SPEED_TO_CHASSIS_SPEED_Forward *
            chassis_move_update->motor_chassis[i].chassis_motor_measure->angle_speed;

        /* ���ٶ� = �ٶ� PID ΢���� �� ��������  (m/s?)  */
        chassis_move_update->motor_chassis[i].accel =
            chassis_move_update->motor_speed_pid[i].Dbuf[0] *
            CHASSIS_CONTROL_FREQUENCE;
    }
/*
  ���������
        0	��ǰ��	FL
        1	��ǰ��	FR  
        2	�����	BL
        3	�Һ���	BR
*/
/* ---- ��һ�����������ʵ����ν���������̿����ò�����δ��� ---- */
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
    /*----������ң���������ϲ���Ƶ���Ϊģʽ������chassis_behaviour����v_Forward_set,v_Move_set,w_Rotate_set ----*/
    chassis_behaviour_control_set(&v_Forward_set, &v_Move_set, &w_Rotate_set, chassis_move_control);

    /* ---- ����������Щ�����ģʽ�����䣬���ʾ��Ƕ������һ���װ ----*/

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW){
        //��w_Rotate_set�� ����ת�ٶȿ���
            /* 1. �޷� */
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

        /* 3. ���˲����д�ؽṹ�壬�ջ������õ�����ֵ */
        chassis_move_control->v_Forward_set = chassis_move_control->chassis_cmd_slow_set_Forward.out;
        chassis_move_control->v_Move_set    = chassis_move_control->chassis_cmd_slow_set_Move.out;
        chassis_move_control->w_Rotate_set  = chassis_move_control->chassis_cmd_slow_set_Rotate.out;
    }else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW){
        //��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->v_Forward_set = v_Forward_set;
        chassis_move_control->v_Move_set = v_Move_set;
        chassis_move_control->w_Rotate_set = w_Rotate_set;
        chassis_move_control-> chassis_cmd_slow_set_Forward.out=0.0f;   //ʹ��һ�׵�ͨ�˲������趨ֵ
        chassis_move_control->chassis_cmd_slow_set_Move.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_Rotate.out =0.0f;
    }
}

/*
  ���������
        0	��ǰ��	FL
        1	��ǰ��	FR  
        2	�����	BL
        3	�Һ���	BR
*/
/* ---- �˶�ѧ�����õ����������Ŀ���ٶ� ---- */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{                   
    const fp32 r = MOTOR_DISTANCE_TO_CENTER;            

    wheel_speed[0] = (-vx_set + vy_set + wz_set * r) ;   // FL
    wheel_speed[1] = ( vx_set + vy_set + wz_set * r) ;   // FR
    wheel_speed[2] = ( vx_set - vy_set + wz_set * r) ;   // BL
    wheel_speed[3] = (-vx_set - vy_set + wz_set * r) ;   // BR
}


/* ---- ���бջ�����*/
void chassis_control_loop(chassis_move_t *chassis_move_control_loop){
    fp32 max_vector = 0.0f;      // ����ٶ��޷�
    fp32 vector_rate = 0.0f;     // ͳһ����ϵ������ĳ������ٶȹ���ʱ��ͬһ����
    fp32 temp = 0.0f;            // ��ʱ����
    fp32 wheel_speed[4] = {0};   // 4 ������Ŀ�����ٶ� [m/s]
    uint8_t i = 0;

    /* ---- �˶�ѧ�����õ������Ŀ���ٶ� ---- */
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->v_Forward_set,
                                          chassis_move_control_loop->v_Move_set,
                                          chassis_move_control_loop->w_Rotate_set,
                                          wheel_speed);

    /* ---- ��ʽģʽ ---- */
    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0; i < 4; i++)
        {
            /* ----  ���ٱ��� ---- */
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

    /* ----  ���ٱ��� ---- */
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i]; // Ŀ���ٶ�
        //ȡ������ٶ����ֵ
        temp = fabsf(wheel_speed[i]);
        if (temp > max_vector) max_vector = temp;                              // �ҳ����max_vector
    }
    //��������ӵ��ٶȴ����޷�����ͳһ����
    if (max_vector > MAX_WHEEL_SPEED)            
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector; // ͳһ����ϵ�������ϵ���Ե��̴ﵽĿ���ٶ�Ӱ����С
        for (i = 0; i < 4; i++)
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }
    /* ---- pid���Ƶõ����ֵ ---- */
    for (i = 0; i < 4; i++)
    {
        //����Ĳ�����m/s���������Ǹ�λ�������ֵ��Ӧ�ٶ�4.2*10��**-4��,
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
                 chassis_move_control_loop->motor_chassis[i].speed,      // ������m/s��
                 chassis_move_control_loop->motor_chassis[i].speed_set); // Ŀ�꣨m/s��
    }
    //������д�ӡ��5�������������������� vx_set,vy_set,vw_set,����1��ʵ���ٶȣ�����1��Ŀ���ٶ�
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
    /* ---- ��������---- */
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current =
            (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
