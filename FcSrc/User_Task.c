
#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include <string.h>


/************************************************************
 * 一、OpenMV全局变量
 ************************************************************/

/*
 * OpenMV识别得到的线路角度
 *
 * > 0 ：线路向右偏
 * < 0 ：线路向左偏
 */
extern int16_t g_openmv_angle;

/*
 * 线路状态
 *
 * 0 = 没检测到线路
 * 1 = 检测到线路
 */
extern uint8_t g_openmv_line_state;

/*
 * 下一拐弯方向
 *
 * 2 = 左转
 * 3 = 右转
 */
extern uint8_t g_openmv_next_direction;

/*
 * 交点/角点检测标志
 *
 * 0 = 没检测到交点
 * 1 = 检测到交点
 */
extern uint8_t g_openmv_cross_flag;

/*
 * 交点X方向误差
 */
extern int16_t g_openmv_cross_x;

/*
 * 交点Y方向误差
 */
extern int16_t g_openmv_cross_y;

/************************************************************
 * 二、飞控状态变量
 ************************************************************/

extern u8 fc_condition;

/*
 * 已完成的转弯次数
 */
extern u8 turn_count;


/************************************************************
 * 三、OpenMV数据帧状态
 ************************************************************/

extern volatile _Bool _WeHaveGotOpenMvOneFrame;
/************************************************************
 * 四、角度PID参数
 ************************************************************/

#define ANGLE_PID_KP             1.0f
#define ANGLE_PID_KI             0.00f
#define ANGLE_PID_KD             0.20f

/*
 * PID最大输出
 */
#define ANGLE_PID_MAX            30.0f

/*
 * 积分最大值
 */
#define ANGLE_PID_I_MAX          30.0f

/*
 * 角度死区
 */
#define ANGLE_DEAD_ZONE          2
/************************************************************
 * 五、转弯参数
 ************************************************************/
/*
 * 每次转90°
 */
#define TURN_ANGLE               90

/*
 * 转弯速度
 */
#define TURN_SPEED               30

/*
 * 转弯前悬停稳定时间
 */
#define HOVER_STABLE_TIME        1000

/*
 * 转弯执行等待时间
 *
 * 注意：
 * 这个时间必须根据你的TurnLeft()/TurnRight()
 * 实际实现进一步调整。
 */
#define TURN_EXECUTE_TIME        10000

/*
 * 一共转4次
 */
#define TOTAL_TURN_TIMES         4

/*
 * 连续检测多少次才确认拐弯
 */
#define TURN_SIGNAL_CONFIRM_COUNT    2

/************************************************************
 * 六、交点对齐参数
 ************************************************************/
/*
 * X方向死区
 */
#define CROSS_X_DEAD_ZONE        30
/*
 * Y方向死区
 */
#define CROSS_Y_DEAD_ZONE        20
/*
 * 每次调整距离
 */
#define CROSS_ADJUST_DIST_CM     2
/*
 * 调整速度
 */
#define CROSS_ADJUST_SPEED_CMPS  20
/*
 * 进入死区后稳定确认时间
 */
#define CROSS_STABLE_TIME_MS     200
/************************************************************
 * 七、巡线运动参数
 ************************************************************/
/*
 * 每次向前移动5cm
 */
#define LINE_MOVE_DISTANCE_CM    5
/*
 * 巡线速度
 */
#define LINE_MOVE_SPEED_CMPS     30
/*
 * PID执行周期
 */
#define LINE_PID_PERIOD_MS       100
/************************************************************
 * 八、角度PID结构体
 ************************************************************/

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;

    float last_error;

    float output;

} OpenMV_AnglePID_t;


/************************************************************
 * 九、PID对象
 ************************************************************/

static OpenMV_AnglePID_t g_angle_pid =
{
    ANGLE_PID_KP,
    ANGLE_PID_KI,
    ANGLE_PID_KD,

    0.0f,
    0.0f,
    0.0f
};


/************************************************************
 * 十、PID复位
 ************************************************************/

static void OpenMV_Angle_PID_Reset(void)
{
    g_angle_pid.integral = 0.0f;

    g_angle_pid.last_error = 0.0f;

    g_angle_pid.output = 0.0f;
}


/************************************************************
 * 十一、角度PID计算
 ************************************************************/

static float OpenMV_Angle_PID_Calculate(float angle)
{
    float error;

    float p;
    float i;
    float d;

    float derivative;

    float output;


    /************************************************
     * 目标角度 = 0°
     ************************************************/

    error = angle;


    /************************************************
     * 角度死区
     ************************************************/

    if((error > -ANGLE_DEAD_ZONE) &&
       (error < ANGLE_DEAD_ZONE))
    {
        error = 0.0f;
    }


    /************************************************
     * P
     ************************************************/

    p = g_angle_pid.kp * error;


    /************************************************
     * I
     ************************************************/

    g_angle_pid.integral += error;


    /*
     * 积分限幅
     */

    if(g_angle_pid.integral > ANGLE_PID_I_MAX)
    {
        g_angle_pid.integral =
            ANGLE_PID_I_MAX;
    }


    if(g_angle_pid.integral < -ANGLE_PID_I_MAX)
    {
        g_angle_pid.integral =
            -ANGLE_PID_I_MAX;
    }


    i = g_angle_pid.ki *
        g_angle_pid.integral;


    /************************************************
     * D
     ************************************************/

    derivative =
        error -
        g_angle_pid.last_error;


    g_angle_pid.last_error =
        error;


    d =
        g_angle_pid.kd *
        derivative;


    /************************************************
     * PID
     ************************************************/

    output =
        p + i + d;


    /************************************************
     * 输出限幅
     ************************************************/

    if(output > ANGLE_PID_MAX)
    {
        output =
            ANGLE_PID_MAX;
    }


    if(output < -ANGLE_PID_MAX)
    {
        output =
            -ANGLE_PID_MAX;
    }


    g_angle_pid.output =
        output;


    return output;
}


/************************************************************
 * 十二、OpenMV视觉巡线
 ************************************************************/

static void OpenMV_Angle_PID_Control(void)
{
    static u16 pid_timer = 0;

    float correction;

    u16 direction;
    /************************************************
     * UserTask默认20ms执行一次
     *
     * 100ms执行一次PID
     ************************************************/
    if(pid_timer < LINE_PID_PERIOD_MS)
    {
        pid_timer += 20;

        return;
    }
    pid_timer = 0;
    /************************************************
     * 没检测到线路
     ************************************************/
    if(g_openmv_line_state == 0)
    {
        OpenMV_Angle_PID_Reset();

        /*
         * 没有线路时保持悬停
         *
         * 如果你希望继续前进，可以删除hover()
         */
        hover();

        return;
    }
    /************************************************
     * PID计算
     ************************************************/

    correction =
        OpenMV_Angle_PID_Calculate(
            (float)g_openmv_angle
        );


    /************************************************
     * 根据PID输出判断运动方向
     *
     * 0°   = 前
     * 90°  = 右
     * 270° = 左
     ************************************************/

    if(correction > 1.0f)
    {
        /*
         * 向右修正
         */

        direction = 90;
    }

    else if(correction < -1.0f)
    {
        /*
         * 向左修正
         */

        direction = 270;
    }

    else
    {
        /*
         * 直线前进
         */

        direction = 0;
    }


    /************************************************
     * 执行运动
     ************************************************/

    Horizontal_Move(
        LINE_MOVE_DISTANCE_CM,
        LINE_MOVE_SPEED_CMPS,
        direction
    );
}


/************************************************************
 * 十三、自动任务
 ************************************************************/

void UserTask_OneKeyCmd(void)
{
    static u8 one_key_takeoff_f = 1;

    static u8 one_key_land_f = 1;

    static u8 one_key_mission_f = 0;
    static u8 mission_step = 0;

    static u16 time_dly_cnt_ms = 0;


    /********************************************************
     * 拐点连续检测计数
     ********************************************************/

    static u8 turn_signal_cnt = 0;

    static u8 turn_direction = 0;
    static u8 turn_start_flag = 0;


    if(rc_in.no_signal == 0)
    {


        if(rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 &&
           rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            if(one_key_takeoff_f == 0)
            {
                one_key_takeoff_f =
                    OneKey_Takeoff(100);
            }
        }

        else
        {
            one_key_takeoff_f = 0;
        }


        if(rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 &&
           rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            if(one_key_land_f == 0)
            {
                one_key_land_f =
                    OneKey_Land();
            }
        }

        else
        {
            one_key_land_f = 0;
        }


        if(rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 &&
           rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {

            if(one_key_mission_f == 0)
            {
                one_key_mission_f = 1;
                mission_step = 1;
                time_dly_cnt_ms = 0;
                turn_signal_cnt = 0;
                turn_direction = 0;
                turn_start_flag = 0;
                turn_count = 0;
                OpenMV_Angle_PID_Reset();
            }
        }

        else
        {
            one_key_mission_f = 0;
        }
        if(one_key_mission_f == 1)
        {
            switch(mission_step)
            {

                case 1:
                {
                    mission_step +=
                        LX_Change_Mode(3);
                }
                break;

                case 2:
                {
                    mission_step +=
                        FC_Unlock();
                }
                break;

                case 3:
                {
                    if(time_dly_cnt_ms < 2000)
                    {
                        time_dly_cnt_ms += 20;
                    }

                    else
                    {
                        time_dly_cnt_ms = 0;

                        mission_step = 4;
                    }
                }
                break;

                case 4:
                {
                    fc_condition = 3;

                    mission_step +=
                        OneKey_Takeoff(100);
                }
                break;


                case 5:
                {
                    if(time_dly_cnt_ms < 5000)
                    {
                        time_dly_cnt_ms += 20;
                    }

                    else
                    {
                        time_dly_cnt_ms = 0;
                        OpenMV_Angle_PID_Reset();

                        mission_step = 6;
                    }
                }
                break;

                case 6:
                {
                    fc_condition = 1;
                    OpenMV_Angle_PID_Control();
                    if(g_openmv_cross_flag == 1)
                    {
                        OpenMV_Angle_PID_Reset();
                        time_dly_cnt_ms = 0;
                        turn_signal_cnt = 0;
                        mission_step = 7;
                    }
                }
                break;
                case 7:
                {
                    fc_condition = 4;
                    if(hover() == 1)
                    {
                        if(time_dly_cnt_ms < HOVER_STABLE_TIME)
                        {
                            time_dly_cnt_ms += 20;
                        }

                        else
                        {
                            time_dly_cnt_ms = 0;
                            turn_signal_cnt = 0;
                            mission_step = 8;
                        }
                    }
                }
                break;

                case 8:
                {
                    fc_condition = 8;


                    //////////////////////////////////////////////////
                    // 没有检测到交点
                    //////////////////////////////////////////////////

                    if(g_openmv_cross_flag == 0)
                    {
                        time_dly_cnt_ms = 0;

                        turn_signal_cnt = 0;

                        hover();

                        break;
                    }


                    //////////////////////////////////////////////////
                    // 第一优先级：
                    // X方向校准
                    //////////////////////////////////////////////////

                    if(g_openmv_cross_x > CROSS_X_DEAD_ZONE)
                    {
                        time_dly_cnt_ms = 0;

                        turn_signal_cnt = 0;
                        Horizontal_Move(
                            CROSS_ADJUST_DIST_CM,
                            CROSS_ADJUST_SPEED_CMPS,
                            90
                        );

                        break;
                    }
                    else if(g_openmv_cross_x < -CROSS_X_DEAD_ZONE)
                    {
                        time_dly_cnt_ms = 0;

                        turn_signal_cnt = 0;
                        Horizontal_Move(
                            CROSS_ADJUST_DIST_CM,
                            CROSS_ADJUST_SPEED_CMPS,
                            270
                        );

                        break;
                    }
                    if(g_openmv_cross_y > CROSS_Y_DEAD_ZONE)
                    {
                        time_dly_cnt_ms = 0;

                        turn_signal_cnt = 0;
                        Horizontal_Move(
                            CROSS_ADJUST_DIST_CM,
                            CROSS_ADJUST_SPEED_CMPS,
                            0
                        );

                        break;
                    }


                    else if(g_openmv_cross_y < -CROSS_Y_DEAD_ZONE)
                    {
                        time_dly_cnt_ms = 0;

                        turn_signal_cnt = 0;
                        Horizontal_Move(
                            CROSS_ADJUST_DIST_CM,
                            CROSS_ADJUST_SPEED_CMPS,
                            180
                        );

                        break;
                    }
                    hover();
                    if(g_openmv_next_direction == 2 ||
                       g_openmv_next_direction == 3)
                    {
                        if(turn_signal_cnt == 0)
                        {
                            turn_direction =
                                g_openmv_next_direction;

                            turn_signal_cnt = 1;
                        }
                        else if(g_openmv_next_direction ==
                                turn_direction)
                        {
                            turn_signal_cnt++;
                        }
                        else
                        {
                            turn_direction =
                                g_openmv_next_direction;

                            turn_signal_cnt = 1;
                        }
                        if(turn_signal_cnt >=
                           TURN_SIGNAL_CONFIRM_COUNT)
                        {
                            turn_signal_cnt = 0;
                            time_dly_cnt_ms = 0;
                            turn_start_flag = 0;
                            mission_step = 9;
                        }
                    }

                    else
                    {
                        turn_signal_cnt = 0;
                    }
                }
                break;
                case 9:
                {
                    if(turn_start_flag == 0)
                    {
                        if(turn_direction == 2)
                        {
                            fc_condition = 2;


                            if(TurnLeft(
                                    TURN_ANGLE,
                                    TURN_SPEED) == 1)
                            {
                                turn_start_flag = 1;

                                time_dly_cnt_ms = 0;
                            }
                        }
                        else if(turn_direction == 3)
                        {
                            fc_condition = 5;


                            if(TurnRight(
                                    TURN_ANGLE,
                                    TURN_SPEED) == 1)
                            {
                                turn_start_flag = 1;

                                time_dly_cnt_ms = 0;
                            }
                        }
                        else
                        {
                            turn_start_flag = 0;

                            turn_signal_cnt = 0;

                            mission_step = 6;
                        }
                    }
                    else
                    {
                        if(time_dly_cnt_ms < TURN_EXECUTE_TIME)
                        {
                            time_dly_cnt_ms += 20;
                        }

                        else
                        {
                            time_dly_cnt_ms = 0;
                            turn_start_flag = 0;
                            turn_count++;
                            OpenMV_Angle_PID_Reset();
                            turn_direction = 0;

                            if(turn_count >= TOTAL_TURN_TIMES)
                            {

                                mission_step = 10;
                            }

                            else
                            {

                                mission_step = 6;
                            }
                        }
                    }
                }
                break;
                case 10:
                {
                    fc_condition = 6;

                    OneKey_Land();
                }
                break;
                default:
                {
                    mission_step = 0;

                    time_dly_cnt_ms = 0;

                    turn_signal_cnt = 0;

                    turn_direction = 0;

                    turn_start_flag = 0;

                    turn_count = 0;

                    OpenMV_Angle_PID_Reset();
                }
                break;
            }
        }

        else
        {
            mission_step = 0;

            time_dly_cnt_ms = 0;

            turn_signal_cnt = 0;

            turn_direction = 0;

            turn_start_flag = 0;

            turn_count = 0;

            OpenMV_Angle_PID_Reset();
        }
    }
}
