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
 *
 * 本版本不再使用
 */
extern int16_t g_openmv_cross_x;


/*
 * 交点Y方向误差
 *
 * 本版本不再使用
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

#define ANGLE_PID_KP 1.0f
#define ANGLE_PID_KI 0.00f
#define ANGLE_PID_KD 0.20f


/*
 * PID最大输出
 */
#define ANGLE_PID_MAX 30.0f


/*
 * 积分最大值
 */
#define ANGLE_PID_I_MAX 30.0f


/*
 * 角度死区
 */
#define ANGLE_DEAD_ZONE 2


/************************************************************
 * 五、转弯参数
 ************************************************************/

/*
 * 每次转90°
 */
#define TURN_ANGLE 90


/*
 * 转弯速度
 */
#define TURN_SPEED 30


/*
 * 转弯执行等待时间
 *
 * 注意：
 * 这个时间必须根据你的TurnLeft()/TurnRight()
 * 实际实现进一步调整。
 */
#define TURN_EXECUTE_TIME 10000


/*
 * 一共转4次
 */
#define TOTAL_TURN_TIMES 4


/*
 * 连续检测多少次才确认拐弯
 *
 * 2次
 */
#define TURN_SIGNAL_CONFIRM_COUNT 2


/************************************************************
 * 六、拐点前进参数
 ************************************************************/

/*
 * 检测到拐点后：
 *
 * 先向前走10cm
 */
#define CROSS_FORWARD_DISTANCE_CM 10


/*
 * 检测到拐点后向前运动速度
 */
#define CROSS_FORWARD_SPEED_CMPS 30


/*
 * 转弯完成后：
 *
 * 再向前走20cm
 */
#define AFTER_TURN_FORWARD_DISTANCE_CM 20


/*
 * 转弯完成后向前运动速度
 */
#define AFTER_TURN_FORWARD_SPEED_CMPS 30


/************************************************************
 * 七、巡线运动参数
 ************************************************************/

/*
 * 每次向前移动5cm
 */
#define LINE_MOVE_DISTANCE_CM 3


/*
 * 巡线速度
 */
#define LINE_MOVE_SPEED_CMPS 30


/*
 * PID执行周期
 */
#define LINE_PID_PERIOD_MS 100


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

    if ((error > -ANGLE_DEAD_ZONE) &&
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

    if (g_angle_pid.integral > ANGLE_PID_I_MAX)
    {
        g_angle_pid.integral =
            ANGLE_PID_I_MAX;
    }

    if (g_angle_pid.integral < -ANGLE_PID_I_MAX)
    {
        g_angle_pid.integral =
            -ANGLE_PID_I_MAX;
    }


    i =
        g_angle_pid.ki *
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

    if (output > ANGLE_PID_MAX)
    {
        output =
            ANGLE_PID_MAX;
    }


    if (output < -ANGLE_PID_MAX)
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

    if (pid_timer < LINE_PID_PERIOD_MS)
    {
        pid_timer += 20;

        return;
    }


    pid_timer = 0;


    /************************************************
     * 没检测到线路
     ************************************************/

    if (g_openmv_line_state == 0)
    {
        OpenMV_Angle_PID_Reset();


        /*
         * 没有线路时保持悬停
         */

        hover();


        return;
    }


    /************************************************
     * PID计算
     ************************************************/

    correction =
        OpenMV_Angle_PID_Calculate(
            (float)g_openmv_angle);


    /************************************************
     * 根据PID输出判断运动方向
     *
     * 0°   = 前
     * 90°  = 右
     * 270° = 左
     ************************************************/

    if (correction > 1.0f)
    {
        /*
         * 向右修正
         */

        direction = 90;
    }

    else if (correction < -1.0f)
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
        direction);
}


/************************************************************
 * 十三、自动任务
 ************************************************************/

void UserTask_OneKeyCmd(void)
{
    /*
     * 一键起飞标志
     */
    static u8 one_key_takeoff_f = 1;


    /*
     * 一键降落标志
     */
    static u8 one_key_land_f = 1;


    /*
     * 自动任务标志
     */
    static u8 one_key_mission_f = 0;


    /*
     * 自动任务状态
     */
    static u8 mission_step = 0;


    /*
     * 通用计时器
     */
    static u16 time_dly_cnt_ms = 0;


    /********************************************************
     * 拐点连续检测计数
     ********************************************************/

    static u8 turn_signal_cnt = 0;


    /*
     * 当前转弯方向
     *
     * 2 = 左
     * 3 = 右
     */
    static u8 turn_direction = 0;


    /*
     * 转弯开始标志
     */
    static u8 turn_start_flag = 0;


    /********************************************************
     * 遥控器有信号
     ********************************************************/

    if (rc_in.no_signal == 0)
    {

        /****************************************************
         * 一键起飞
         *
         * CH6：
         *
         * 1300 ~ 1700
         ****************************************************/

        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 &&
            rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            if (one_key_takeoff_f == 0)
            {
                one_key_takeoff_f =
                    OneKey_Takeoff(100);
            }
        }

        else
        {
            one_key_takeoff_f = 0;
        }


        /****************************************************
         * 一键降落
         *
         * CH6：
         *
         * 800 ~ 1200
         ****************************************************/

        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 &&
            rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            if (one_key_land_f == 0)
            {
                one_key_land_f =
                    OneKey_Land();
            }
        }

        else
        {
            one_key_land_f = 0;
        }


        /****************************************************
         * 自动任务
         *
         * CH6：
         *
         * 1700 ~ 2200
         ****************************************************/

        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 &&
            rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {

            if (one_key_mission_f == 0)
            {
                /*
                 * 启动自动任务
                 */

                one_key_mission_f = 1;


                /*
                 * 从STEP1开始
                 */

                mission_step = 1;


                /*
                 * 清零计时器
                 */

                time_dly_cnt_ms = 0;


                /*
                 * 清零转弯检测
                 */

                turn_signal_cnt = 0;


                /*
                 * 清零转弯方向
                 */

                turn_direction = 0;


                /*
                 * 清零转弯启动标志
                 */

                turn_start_flag = 0;


                /*
                 * 清零转弯次数
                 */

                turn_count = 0;


                /*
                 * PID复位
                 */

                OpenMV_Angle_PID_Reset();
            }
        }

        else
        {
            one_key_mission_f = 0;
        }


        /****************************************************
         * 自动任务运行
         ****************************************************/

        if (one_key_mission_f == 1)
        {
            switch (mission_step)
            {

            /************************************************
             * STEP 1
             *
             * 切换飞行模式
             ************************************************/

            case 1:
            {
                mission_step +=
                    LX_Change_Mode(3);
            }
            break;


            /************************************************
             * STEP 2
             *
             * 解锁
             ************************************************/

            case 2:
            {
                mission_step +=
                    FC_Unlock();
            }
            break;


            /************************************************
             * STEP 3
             *
             * 解锁后等待2秒
             ************************************************/

            case 3:
            {
                if (time_dly_cnt_ms < 2000)
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


            /************************************************
             * STEP 4
             *
             * 起飞到100cm
             ************************************************/

            case 4:
            {
                fc_condition = 3;


                mission_step +=
                    OneKey_Takeoff(100);
            }
            break;


            /************************************************
             * STEP 5
             *
             * 起飞后等待5秒
             ************************************************/

            case 5:
            {
                if (time_dly_cnt_ms < 5000)
                {
                    time_dly_cnt_ms += 20;
                }

                else
                {
                    time_dly_cnt_ms = 0;


                    /*
                     * PID复位
                     */

                    OpenMV_Angle_PID_Reset();


                    /*
                     * 开始视觉巡线
                     */

                    mission_step = 6;
                }
            }
            break;


            /************************************************
             * STEP 6
             *
             * 正常视觉巡线
             *
             * 正常情况下：
             *     OpenMV检测线路
             *     根据线路角度进行修正
             *
             * 检测到拐点后：
             *     不进行X/Y对齐
             *     不停在拐点中心
             *     直接进入STEP7
             ************************************************/

            case 6:
            {
                fc_condition = 1;


                /*
                 * 正常视觉巡线
                 */

                OpenMV_Angle_PID_Control();


                /********************************************
                 * 检测到拐点
                 ********************************************/

                if (g_openmv_cross_flag == 1)
                {
                    /*
                     * 停止PID积分
                     */

                    OpenMV_Angle_PID_Reset();


                    /*
                     * 清零相关状态
                     */

                    time_dly_cnt_ms = 0;

                    turn_signal_cnt = 0;

                    turn_direction = 0;

                    turn_start_flag = 0;


                    /*
                     * 进入：
                     *
                     * 向前10cm
                     */

                    mission_step = 7;
                }
            }
            break;


            /************************************************
             * STEP 7
             *
             * 检测到拐点后
             *
             * 直接向前走10cm
             *
             * 不进行X/Y中心对齐
             ************************************************/

            case 7:
            {
                fc_condition = 4;


                /*
                 * 向前走10cm
                 *
                 * 0° = 前
                 */

                if (Horizontal_Move(
                        CROSS_FORWARD_DISTANCE_CM,
                        CROSS_FORWARD_SPEED_CMPS,
                        0) == 1)
                {
                    /*
                     * 10cm前进完成
                     */

                    time_dly_cnt_ms = 0;

                    turn_signal_cnt = 0;

                    mission_step = 8;
                }
            }
            break;


            /************************************************
             * STEP 8
             *
             * 判断拐弯方向
             *
             * 2 = 左转
             * 3 = 右转
             *
             * 连续检测2次相同方向后确认
             ************************************************/

            case 8:
            {
                fc_condition = 8;


                /********************************************
                 * 判断OpenMV转弯方向
                 ********************************************/

                if (g_openmv_next_direction == 2 ||
                    g_openmv_next_direction == 3)
                {

                    /****************************************
                     * 第一次检测到有效方向
                     ****************************************/

                    if (turn_signal_cnt == 0)
                    {
                        turn_direction =
                            g_openmv_next_direction;

                        turn_signal_cnt = 1;
                    }


                    /****************************************
                     * 后续方向相同
                     ****************************************/

                    else if (g_openmv_next_direction ==
                             turn_direction)
                    {
                        turn_signal_cnt++;
                    }


                    /****************************************
                     * 方向发生变化
                     ****************************************/

                    else
                    {
                        turn_direction =
                            g_openmv_next_direction;

                        turn_signal_cnt = 1;
                    }


                    /****************************************
                     * 连续确认完成
                     ****************************************/

                    if (turn_signal_cnt >=
                        TURN_SIGNAL_CONFIRM_COUNT)
                    {
                        /*
                         * 清零确认次数
                         */

                        turn_signal_cnt = 0;


                        /*
                         * 清零计时
                         */

                        time_dly_cnt_ms = 0;


                        /*
                         * 准备开始转弯
                         */

                        turn_start_flag = 0;


                        /*
                         * 进入STEP9
                         */

                        mission_step = 9;
                    }
                }

                else
                {
                    /*
                     * 当前没有有效方向
                     */

                    turn_signal_cnt = 0;
                }
            }
            break;


            /************************************************
             * STEP 9
             *
             * 执行90°转弯
             *
             * 2 = 左转90°
             * 3 = 右转90°
             ************************************************/

            case 9:
            {

                /********************************************
                 * 尚未开始转弯
                 ********************************************/

                if (turn_start_flag == 0)
                {

                    /****************************************
                     * 左转90°
                     ****************************************/

                    if (turn_direction == 2)
                    {
                        fc_condition = 2;


                        if (TurnLeft(
                                TURN_ANGLE,
                                TURN_SPEED) == 1)
                        {
                            /*
                             * 转弯动作已经开始
                             */

                            turn_start_flag = 1;


                            time_dly_cnt_ms = 0;
                        }
                    }


                    /****************************************
                     * 右转90°
                     ****************************************/

                    else if (turn_direction == 3)
                    {
                        fc_condition = 5;


                        if (TurnRight(
                                TURN_ANGLE,
                                TURN_SPEED) == 1)
                        {
                            /*
                             * 转弯动作已经开始
                             */

                            turn_start_flag = 1;


                            time_dly_cnt_ms = 0;
                        }
                    }


                    /****************************************
                     * 没有有效转弯方向
                     ****************************************/

                    else
                    {
                        turn_start_flag = 0;

                        turn_signal_cnt = 0;

                        mission_step = 6;
                    }
                }


                /********************************************
                 * 已经开始转弯
                 ********************************************/

                else
                {

                    /****************************************
                     * 等待转弯执行完成
                     ****************************************/

                    if (time_dly_cnt_ms <
                        TURN_EXECUTE_TIME)
                    {
                        time_dly_cnt_ms += 20;
                    }


                    /****************************************
                     * 转弯完成
                     ****************************************/

                    else
                    {
                        /*
                         * 清零计时器
                         */

                        time_dly_cnt_ms = 0;


                        /*
                         * 清除转弯开始标志
                         */

                        turn_start_flag = 0;


                        /*
                         * 转弯次数+1
                         */

                        turn_count++;


                        /*
                         * PID复位
                         */

                        OpenMV_Angle_PID_Reset();


                        /*
                         * 清除转弯方向
                         */

                        turn_direction = 0;


                        /*
                         * 进入STEP10
                         *
                         * 转完90°后
                         * 再向前走20cm
                         */

                        mission_step = 10;
                    }
                }
            }
            break;


            /************************************************
             * STEP 10
             *
             * 转弯完成后
             *
             * 继续向前直走20cm
             *
             * 走完之后：
             *
             * 如果还没有完成4次转弯
             *     回到STEP6继续巡线
             *
             * 如果已经完成4次转弯
             *     进入STEP11降落
             ************************************************/

            case 10:
            {
                fc_condition = 1;


                /********************************************
                 * 向前直走20cm
                 *
                 * 0° = 前
                 ********************************************/

                if (Horizontal_Move(
                        AFTER_TURN_FORWARD_DISTANCE_CM,
                        AFTER_TURN_FORWARD_SPEED_CMPS,
                        0) == 1)
                {
                    /*
                     * 20cm前进完成
                     */

                    time_dly_cnt_ms = 0;


                    /*
                     * PID复位
                     */

                    OpenMV_Angle_PID_Reset();


                    /****************************************
                     * 判断是否完成4次转弯
                     ****************************************/

                    if (turn_count >= TOTAL_TURN_TIMES)
                    {
                        /*
                         * 4次转弯全部完成
                         *
                         * 进入降落
                         */

                        mission_step = 11;
                    }

                    else
                    {
                        /*
                         * 还没有完成4次转弯
                         *
                         * 回到视觉巡线
                         */

                        mission_step = 6;
                    }
                }
            }
            break;


            /************************************************
             * STEP 11
             *
             * 自动降落
             ************************************************/

            case 11:
            {
                fc_condition = 6;


                OneKey_Land();
            }
            break;


            /************************************************
             * 默认状态
             ************************************************/

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


        /****************************************************
         * 自动任务没有启动
         ****************************************************/

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