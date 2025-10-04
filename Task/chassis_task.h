#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__


#include "struct_typedef.h"
#include "pid.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "Serial_test.h"
#include "User_lib.h"


// #include "chassis_task_assign.h"


//任务开始延迟时间
#define CHASSIS_TASK_INIT_TIME 357

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2

/*----------------------- 通道对应任务宏定义 -----------------------*/
/*----通道3对应的是左摇杆的上下值，从上到下递减，用作前进----*/
#define CHASSIS_Forward_CHANNEL 3             

/*----通道0对应的是右摇杆的左右值，从左到右递增，用作平移----*/
#define CHASSIS_Move_CHANNEL 0

/*----通道二对应左摇杆的左右值，从左到右递增-，用于控制旋转，正为逆时针旋转---*/
#define CHASSIS_Rotate_CHANNEL 1        

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 1

/*----------------------- 通道值对应的速度转换，前进最快大概4m/s，平移3.3m/s -----------------------*/
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_Forward_RC_SEN 0.0027f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_Move_RC_SEN 0.0027f

#define CHASSIS_Rotate_RC_SEN  0.003f

/*----------------------- 优化方向的宏定义，加入一阶滤波和摇杆死区 -----------------------*/

/*----一阶低通的“平滑时间常数 τ----*/
#define CHASSIS_ACCEL_Forward_NUM 0.1666666667f
#define CHASSIS_ACCEL_Move_NUM 0.3333333333f
#define CHASSIS_ACCEL_ROTATE_NUM 0.1f   // τ = 100 ms

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

/*----------------------- 关于正逆解算几个重要的宏定义 -----------------------*/

//电机报文反馈rpm对应轮子的线速度值m/s
#define MOTOR_SPEED_TO_CHASSIS_SPEED_Forward 0.000420f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_Move 0.000420f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_Rotate 0.00168f        //这个单位是rad/s

//控制频率，求解加速度
#define CHASSIS_CONTROL_FREQUENCE         500.0f

//轮中心到底盘中心的距离
#define MOTOR_DISTANCE_TO_CENTER        0.25f

//轮子直径，猜测
#define WHEEL_DIAMETER          0.1525f
//轮子半径
#define WHEEL_RADIUS            (WHEEL_DIAMETER * 0.5f)     

/*----------------------- 关于底盘与电机之间的一些参数设计 -----------------------*/

#define MAX_MOTOR_RPM 6000                  //  防止过载
#define MAX_MOTOR_CURRENT 15000             //  15 A

#define MAX_CHASSIS_Forward_Speed   2.4       //  前进最快2.4m/s
#define MAX_CHASSIS_Move_Speed      2.4     //  平移最快2.4m/s
#define MAX_CHASSIS_Rotate_Speed    5       //  旋转最快5rad/s    
#define MAX_WHEEL_SPEED 3.5f

/*----------------------- 关于电机pid参数的设计，用于初始化，4个电机共用一套pid -----------------------*/
/* ---- 参数设计：误差值在0~2之间，最大值在15000， */
#define M3508_MOTOR_SPEED_PID_KP 15000.0f
#define M3508_MOTOR_SPEED_PID_KI 100.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 3000.0f


/*----------------------- 关于底盘控制的结构体 -----------------------*/

/*----用于控制底盘电机运动状态----*/
typedef struct
{
  const  motor_measure_t *chassis_motor_measure;      //反馈的数据，原始数据
  fp32 accel;                                         //计算出来的加速度
  fp32 speed;                                         //实际速度，m/s
  fp32 speed_set;                                     //目标速度      
  int16_t give_current;                               //给定换算后的电流，最后给motor函数输出
} chassis_motor_t;

/*----------------------- 关于底盘控制的结构体 -----------------------*/

/*----定义底盘两种模式枚举类型----*/
typedef enum{
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //对应上层行为的停止与遥控器控制
  CHASSIS_VECTOR_RAW,                 //无任何闭环，进行调试
} chassis_mode_e;


/*----控制移动的结构体----*/
typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针

  chassis_motor_t motor_chassis[4];          //chassis motor data.底盘电机数据

  pid_type_def motor_speed_pid[4];           //motor speed PID.底盘电机速度pid，每个电机对应一个pid

  chassis_mode_e chassis_mode;               //底盘控制状态机
  chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机

  first_order_filter_type_t chassis_cmd_slow_set_Forward;   //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_Move;      //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_Rotate;

  /*----运动学正解算算出来的实际值----*/
  fp32 v_Forward;                          //底盘速度 前进方向 前为正，单位 m/s，这里与rpm进行转换了
  fp32 v_Move;                          //底盘速度 左右方向 左为正  单位 m/s
  fp32 w_Rotate;                          //底盘旋转角速度，逆时针为正 单位 rad/s

  /*----v_Forward_set,v_Move_set,w_Rotate_set来自遥控器*/
  fp32 v_Forward_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 v_Move_set;                         //底盘设定速度 左右方向 左为负，单位 m/s
  fp32 w_Rotate_set;                       //底盘设定旋速度，逆时转角针为正 单位 rad/s

  /*----设定移动的最大值，初始化的时候上宏定义----*/
  fp32 v_Forward_max_speed;             //前进方向最大速度 单位m/s
  fp32 w_Rotate_max_speed;              //旋转方向最大速度 单位rad/s
  fp32 v_Move_max_speed;                //平移最大速度

} chassis_move_t;


/*----------------------- 对外结构声明 -----------------------*/

/*----对外接口，控制任务的核心函数*/
extern void chassis_task(void *pvParameters);

/*---- 根据遥控器通道值，解算出运动值*/
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set,chassis_move_t *chassis_move_rc_to_vector);

#endif
