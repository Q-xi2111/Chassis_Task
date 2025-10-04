#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__


#include "struct_typedef.h"
#include "pid.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "Serial_test.h"
#include "User_lib.h"


// #include "chassis_task_assign.h"


//����ʼ�ӳ�ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2

/*----------------------- ͨ����Ӧ����궨�� -----------------------*/
/*----ͨ��3��Ӧ������ҡ�˵�����ֵ�����ϵ��µݼ�������ǰ��----*/
#define CHASSIS_Forward_CHANNEL 3             

/*----ͨ��0��Ӧ������ҡ�˵�����ֵ�������ҵ���������ƽ��----*/
#define CHASSIS_Move_CHANNEL 0

/*----ͨ������Ӧ��ҡ�˵�����ֵ�������ҵ���-�����ڿ�����ת����Ϊ��ʱ����ת---*/
#define CHASSIS_Rotate_CHANNEL 1        

//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 1

/*----------------------- ͨ��ֵ��Ӧ���ٶ�ת����ǰ�������4m/s��ƽ��3.3m/s -----------------------*/
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_Forward_RC_SEN 0.0027f

//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_Move_RC_SEN 0.0027f

#define CHASSIS_Rotate_RC_SEN  0.003f

/*----------------------- �Ż�����ĺ궨�壬����һ���˲���ҡ������ -----------------------*/

/*----һ�׵�ͨ�ġ�ƽ��ʱ�䳣�� ��----*/
#define CHASSIS_ACCEL_Forward_NUM 0.1666666667f
#define CHASSIS_ACCEL_Move_NUM 0.3333333333f
#define CHASSIS_ACCEL_ROTATE_NUM 0.1f   // �� = 100 ms

//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//ҡ������
#define CHASSIS_RC_DEADLINE 10

/*----------------------- ����������㼸����Ҫ�ĺ궨�� -----------------------*/

//������ķ���rpm��Ӧ���ӵ����ٶ�ֵm/s
#define MOTOR_SPEED_TO_CHASSIS_SPEED_Forward 0.000420f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_Move 0.000420f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_Rotate 0.00168f        //�����λ��rad/s

//����Ƶ�ʣ������ٶ�
#define CHASSIS_CONTROL_FREQUENCE         500.0f

//�����ĵ��������ĵľ���
#define MOTOR_DISTANCE_TO_CENTER        0.25f

//����ֱ�����²�
#define WHEEL_DIAMETER          0.1525f
//���Ӱ뾶
#define WHEEL_RADIUS            (WHEEL_DIAMETER * 0.5f)     

/*----------------------- ���ڵ�������֮���һЩ������� -----------------------*/

#define MAX_MOTOR_RPM 6000                  //  ��ֹ����
#define MAX_MOTOR_CURRENT 15000             //  15 A

#define MAX_CHASSIS_Forward_Speed   2.4       //  ǰ�����2.4m/s
#define MAX_CHASSIS_Move_Speed      2.4     //  ƽ�����2.4m/s
#define MAX_CHASSIS_Rotate_Speed    5       //  ��ת���5rad/s    
#define MAX_WHEEL_SPEED 3.5f

/*----------------------- ���ڵ��pid��������ƣ����ڳ�ʼ����4���������һ��pid -----------------------*/
/* ---- ������ƣ����ֵ��0~2֮�䣬���ֵ��15000�� */
#define M3508_MOTOR_SPEED_PID_KP 15000.0f
#define M3508_MOTOR_SPEED_PID_KI 100.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 3000.0f


/*----------------------- ���ڵ��̿��ƵĽṹ�� -----------------------*/

/*----���ڿ��Ƶ��̵���˶�״̬----*/
typedef struct
{
  const  motor_measure_t *chassis_motor_measure;      //���������ݣ�ԭʼ����
  fp32 accel;                                         //��������ļ��ٶ�
  fp32 speed;                                         //ʵ���ٶȣ�m/s
  fp32 speed_set;                                     //Ŀ���ٶ�      
  int16_t give_current;                               //���������ĵ���������motor�������
} chassis_motor_t;

/*----------------------- ���ڵ��̿��ƵĽṹ�� -----------------------*/

/*----�����������ģʽö������----*/
typedef enum{
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //��Ӧ�ϲ���Ϊ��ֹͣ��ң��������
  CHASSIS_VECTOR_RAW,                 //���καջ������е���
} chassis_mode_e;


/*----�����ƶ��Ľṹ��----*/
typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��

  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������

  pid_type_def motor_speed_pid[4];           //motor speed PID.���̵���ٶ�pid��ÿ�������Ӧһ��pid

  chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��

  first_order_filter_type_t chassis_cmd_slow_set_Forward;   //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_Move;      //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_Rotate;

  /*----�˶�ѧ�������������ʵ��ֵ----*/
  fp32 v_Forward;                          //�����ٶ� ǰ������ ǰΪ������λ m/s��������rpm����ת����
  fp32 v_Move;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 w_Rotate;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

  /*----v_Forward_set,v_Move_set,w_Rotate_set����ң����*/
  fp32 v_Forward_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 v_Move_set;                         //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 w_Rotate_set;                       //�����趨���ٶȣ���ʱת����Ϊ�� ��λ rad/s

  /*----�趨�ƶ������ֵ����ʼ����ʱ���Ϻ궨��----*/
  fp32 v_Forward_max_speed;             //ǰ����������ٶ� ��λm/s
  fp32 w_Rotate_max_speed;              //��ת��������ٶ� ��λrad/s
  fp32 v_Move_max_speed;                //ƽ������ٶ�

} chassis_move_t;


/*----------------------- ����ṹ���� -----------------------*/

/*----����ӿڣ���������ĺ��ĺ���*/
extern void chassis_task(void *pvParameters);

/*---- ����ң����ͨ��ֵ��������˶�ֵ*/
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set,chassis_move_t *chassis_move_rc_to_vector);

#endif
