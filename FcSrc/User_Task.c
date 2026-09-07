#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include <string.h>

extern int16_t g_openmv_angle;
extern uint8_t g_openmv_line_state;
extern  uint8_t g_openmv_next_direction;
extern uint8_t g_openmv_cross_flag;
extern int16_t g_openmv_cross_x; 
extern int16_t g_openmv_cross_y;  // 新增：拐点横向误差


extern u8 fc_condition ;
extern u8 turn_count ;
extern volatile uint8_t g_openmv_red_count;
extern volatile uint8_t g_openmv_blue_count;
extern volatile uint8_t g_openmv_green_count;


volatile u8 g_corner_red_count = 0;
volatile u8 g_corner_blue_count = 0;
volatile u8 g_corner_green_count = 0;


extern volatile _Bool _WeHaveGotOpenMvOneFrame;

/************************************************************
 * OpenMV 角度 PID 巡线
 *
 * OpenMV 数据：
 *
 * [10] angle LOW
 * [11] angle HIGH
 *
 * STM32 接收后：
 *
 * g_openmv_angle
 *
 * 例如：
 *
 *   g_openmv_angle =  20
 *      表示线路向右偏20°
 *
 *   g_openmv_angle = -20
 *      表示线路向左偏20°
 *
 *   g_openmv_angle = 0
 *      表示线路基本正向
 *
 *
 * PID输出：
 *
 *   正数 -> 向右修正
 *   负数 -> 向左修正
 *
 * 最后转换成 Horizontal_Move() 的方向角：
 *
 *   0°   = 前进
 *   90°  = 右
 *   270° = 左
 ************************************************************/
/************************************************************
 * 一、角度 PID 参数
 ************************************************************/
/*
 * 第一版建议保守。
 *
 * P：
 * 当前角度偏差越大，修正越大。
 *
 * I：
 * 暂时关闭，防止积分累积造成飞行器持续偏移。
 *
 * D：
 * 抑制角度变化过快。
 */
#define ANGLE_PID_KP             1.0f
#define ANGLE_PID_KI             0.00f
#define ANGLE_PID_KD             0.20f
/*
 * PID最大输出角
 *
 * 例如：
 *
 * PID = +20
 * -> 向右20°
 *
 * PID = -20
 * -> 向左20°
 *
 * 最大限制 ±30°
 */
#define ANGLE_PID_MAX            30.0f
/*
 * 积分最大值
 */
#define ANGLE_PID_I_MAX          30.0f
/*
 * 角度死区
 *
 * 小于2°认为已经基本对准。
 */
#define ANGLE_DEAD_ZONE          2

/* ========== 新增：转弯流程参数 ========== */
#define TURN_ANGLE               90     // 单次转弯角度（°）
#define TURN_SPEED               30     // 转弯角速度（°/s）
#define HOVER_STABLE_TIME        1000   // 悬停稳定等待时间（ms）
#define CORNER_CONFIRM_TIME      500    // 拐角位置确认时间（ms）
#define TURN_EXECUTE_TIME        10000   // 转弯执行总时长（ms），含余量
#define TOTAL_TURN_TIMES         4      // 正方形总转弯次数


/* ===== 拐点对准参数 ===== */
#define CROSS_X_DEAD_ZONE        30      // X轴（前后）死区
#define CROSS_Y_DEAD_ZONE        20      // Y轴（左右）死区
#define CROSS_ADJUST_DIST_CM     2      // 每次微调距离（cm）
#define CROSS_ADJUST_SPEED_CMPS  20     // 微调速度（cm/s）
#define CROSS_STABLE_TIME_MS     200    // 稳定确认时间（ms）



/************************************************************
 * 二、视觉运动参数
 ************************************************************/
/*
 * 每次只走5cm。
 *
 * 这样可以：
 *
 * 视觉检测
 * ↓
 * PID计算
 * ↓
 * 飞5cm
 * ↓
 * 再看新的视觉数据
 *
 * 比一次飞很远更安全。
 */
#define LINE_MOVE_DISTANCE_CM    5
#define LINE_MOVE_SPEED_CMPS     30
/*
 * PID执行周期
 *
 * UserTask通常20ms调用一次。
 *
 * 5 × 20ms = 100ms
 */
#define LINE_PID_PERIOD_MS       100
/************************************************************
 * 三、角度 PID 结构体
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
 * 四、PID对象
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
 * 五、PID复位
 ************************************************************/
static void OpenMV_Angle_PID_Reset(void)
{
    g_angle_pid.integral = 0.0f;
    g_angle_pid.last_error = 0.0f;
    g_angle_pid.output = 0.0f;
}
/************************************************************
 * 六、PID计算
 *
 * 输入：
 *
 *     angle
 *
 * 例如：
 *
 *     +10
 *     -20
 *      0
 *
 * 输出：
 *
 *     -30 ~ +30
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
     *
     * 所以：
     *
     * error = 当前角度
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
        g_angle_pid.integral = ANGLE_PID_I_MAX;
    }
    if(g_angle_pid.integral < -ANGLE_PID_I_MAX)
    {
        g_angle_pid.integral = -ANGLE_PID_I_MAX;
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
     * PID输出限幅
     ************************************************/
    if(output > ANGLE_PID_MAX)
    {
        output = ANGLE_PID_MAX;
    }
    if(output < -ANGLE_PID_MAX)
    {
        output = -ANGLE_PID_MAX;
    }
    g_angle_pid.output =
        output;
    return output;
}
/************************************************************
 * 七、OpenMV角度PID控制
 *
 * 这个函数是实际执行飞行纠偏的函数。
 *
 * 流程：
 *
 * OpenMV
 *   ↓
 * g_openmv_angle
 *   ↓
 * PID
 *   ↓
 * correction
 *   ↓
 * 方向角
 *   ↓
 * Horizontal_Move()
 ************************************************************/
static void OpenMV_Angle_PID_Control(void)
{
    static u16 pid_timer = 0;
    float correction;
    u16 direction;
    /************************************************
     * 20ms周期累计
     ************************************************/
    if(pid_timer < LINE_PID_PERIOD_MS)
    {
        pid_timer += 20;
        return;
    }
    pid_timer = 0;
    /************************************************
     * 没检测到直线
     ************************************************/
    if(g_openmv_line_state == 0)
    {
        OpenMV_Angle_PID_Reset();
        return;
    }
    /************************************************
     * 读取OpenMV角度
     ************************************************/
    correction =
        OpenMV_Angle_PID_Calculate(
            (float)g_openmv_angle
        );
    /************************************************
     * 根据PID输出判断方向
     *
     * 0°   = 前进
     * 90°  = 右
     * 270° = 左
     ************************************************/
    if(correction > 1.0f)
    {
        /*
         * 正数
         *
         * 向右修正
         */
        direction = 90;
    }
    else if(correction < -1.0f)
    {
        /*
         * 负数
         *
         * 向左修正
         */
        direction = 270;
    }
    else
    {
        /*
         * 基本不需要左右修正
         *
         * 直接前进
         */
        direction = 0;
    }
    /************************************************
     * 执行运动
     *
     * 注意：
     *
     * 这里不要访问 dt.wait_ck
     *
     * 因为dt不是User_Task.c的全局变量。
     *
     * Horizontal_Move()内部已经处理：
     *
     * if(dt.wait_ck == 0)
     *     发送
     * else
     *     等待
     ************************************************/
    Horizontal_Move(
        LINE_MOVE_DISTANCE_CM,
        LINE_MOVE_SPEED_CMPS,
        direction
    );
}
/************************************************************
 * 八、自动任务
 ************************************************************/
void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    // 一键起飞 / 降落 / 自动巡线
    //////////////////////////////////////////////////////////////////////
    static u8 one_key_takeoff_f = 1;
    static u8 one_key_land_f = 1;
    static u8 one_key_mission_f = 0;
    static u8 mission_step;
    static u16 time_dly_cnt_ms;
	static u8 turn_signal_cnt = 0;  // 左转信号连续检测计数器
	static u8 turn_start_flag = 0;
	static u8 turn_execute_lock = 0;     // 转弯执行锁：1=正在执行转弯，禁止重新判断
	static u8 turn_direction = 0;
	static u8 ball_count_done = 0;
	
	
	// 四个拐角的小球统计
// corner_ball_count[拐角][颜色]
// 颜色：0=红，1=绿，2=黄
// =====================================================
static u8 corner_ball_count[4][3] = {0};
// =====================================================
// 当前拐角5帧统计
// =====================================================
static u8 ball_sample_count = 0;

static u8 max_red_count = 0;
static u8 max_green_count = 0;
static u8 max_blue_count = 0;
volatile u8 g_corner_red_count = 0;
volatile u8 g_corner_blue_count = 0;
volatile u8 g_corner_green_count = 0;
    /********************************************************
     * 判断遥控器是否有信号
     ********************************************************/
    if(rc_in.no_signal == 0)
    {
        //////////////////////////////////////////////////////
        // 一键起飞
        //////////////////////////////////////////////////////
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
        //////////////////////////////////////////////////////
        // 一键降落
        //////////////////////////////////////////////////////
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
        //////////////////////////////////////////////////////
        // 自动任务
        //////////////////////////////////////////////////////
        if(rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 &&
           rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        { 
            if(one_key_mission_f == 0)
{
    one_key_mission_f = 1;
    mission_step = 1;
    time_dly_cnt_ms = 0;

    turn_count = 0;
    turn_signal_cnt = 0;
    turn_start_flag = 0;
    turn_execute_lock = 0;
	turn_direction = 0;
	ball_sample_count = 0;

max_red_count = 0;
max_blue_count = 0;
max_green_count = 0;

for(u8 i = 0; i < 4; i++)
{
    corner_ball_count[i][0] = 0;
    corner_ball_count[i][1] = 0;
    corner_ball_count[i][2] = 0;
}

    OpenMV_Angle_PID_Reset();
}
        }
        else
        {
            one_key_mission_f = 0;
        }
        //////////////////////////////////////////////////////
        // 自动任务执行
        //////////////////////////////////////////////////////
        if(one_key_mission_f == 1)
        {
            switch(mission_step)
            {
                //////////////////////////////////////////////////
                // STEP 0
                //////////////////////////////////////////////////
                case 0:
                {
                    time_dly_cnt_ms = 0;
                    turn_count = 0;
					turn_signal_cnt = 0;  // 复位左转信号计数器
                  //  OpenMV_Angle_PID_Reset();
                }
                break;
                //////////////////////////////////////////////////
                // STEP 1
                //
                // 切换程控模式
                //////////////////////////////////////////////////
                case 1:
                {
                    mission_step +=
                        LX_Change_Mode(3);
                }
                break;
                //////////////////////////////////////////////////
                // STEP 2
                //
                // 解锁
                //////////////////////////////////////////////////
                case 2:
                {
                    mission_step +=
                        FC_Unlock();
                }
                break;
                //////////////////////////////////////////////////
                // STEP 3
                //
                // 等待2秒
                //////////////////////////////////////////////////
                case 3:
                {
                    if(time_dly_cnt_ms < 2000)
                    {
                        time_dly_cnt_ms += 20;
                    }
                    else
                    {
                        time_dly_cnt_ms = 0;
                        mission_step += 1;
                    }
                }
                break;
                //////////////////////////////////////////////////
                // STEP 4
                //
                // 起飞100cm
                //////////////////////////////////////////////////
                case 4:
                {   fc_condition=3;
                    mission_step +=
                        OneKey_Takeoff(100);
                }
                break;
                //////////////////////////////////////////////////
                // STEP 5
                //
                // 起飞后等待5秒
                //////////////////////////////////////////////////
                case 5:
                {
                    if(time_dly_cnt_ms < 5000)
                    {
                        time_dly_cnt_ms += 20;
                    }
                    else
                    {
                        time_dly_cnt_ms = 0;
                      //  OpenMV_Angle_PID_Reset();
                        mission_step += 1;
                    }
                }
                break;
                //////////////////////////////////////////////////
                // STEP 6
                //
                // OpenMV视觉角度PID巡线（直线段）
                // 检测到左转信号后进入转弯流程
                //////////////////////////////////////////////////
                //////////////////////////////////////////////////
// STEP 6
//
// OpenMV视觉角度PID巡线（直线段）
// 连续检测到3次左转信号后进入转弯流程
//////////////////////////////////////////////////
//case 6:
//{
//    // 正常直线巡线
//   // OpenMV_Angle_PID_Control();
//    fc_condition = 1;

//    // 只有没有处于转弯执行状态时，
//    // 才允许检测新的转弯信号
//    if(turn_execute_lock == 0)
//    {
//        // =========================================
//        // 检测左转 / 右转信号
//        // =========================================
//        if(g_openmv_line_state == 1 &&
//           (g_openmv_next_direction == 2 ||
//            g_openmv_next_direction == 3))
//        {
//            turn_signal_cnt++;

//            // 连续检测3次
//            if(turn_signal_cnt >= 3)
//            {
//                // =================================
//                // 锁存本次转弯方向
//                // =================================
//                turn_direction = g_openmv_next_direction;

//                // 开启转弯执行锁
//                turn_execute_lock = 1;

//                // 清零信号计数
//                turn_signal_cnt = 0;

//                // 停止视觉PID
//                OpenMV_Angle_PID_Reset();

//                // 清零计时
//                time_dly_cnt_ms = 0;

//                // 进入悬停
//                mission_step = 7;
//            }
//        }
//        else
//        {
//            turn_signal_cnt = 0;
//        }
//    }
//}
//break;
case 6:
{
   

    // =========================================
    // 先不使用PID，直接向前移动
    // 距离：5cm
    // 速度：20cm/s
    // 方向：0度
    // =========================================
    Horizontal_Move(5, 20, 0);
	
	fc_condition=1;
    // =========================================
    // 检测左转 / 右转信号
    // =========================================
    if(turn_execute_lock == 0)
    {
        if(g_openmv_line_state == 1 &&
           (g_openmv_next_direction == 2 ||
            g_openmv_next_direction == 3))
        {
            turn_signal_cnt++;

            // 连续检测3次
            if(turn_signal_cnt >= 3)
            {
                turn_direction = g_openmv_next_direction;

                turn_execute_lock = 1;

                turn_signal_cnt = 0;

                time_dly_cnt_ms = 0;

                mission_step = 7;
            }
        }
        else
        {
            turn_signal_cnt = 0;
        }
    }
}
break;
                //////////////////////////////////////////////////
                // STEP 7
                //
                // 减速悬停，等待机身稳定
                //////////////////////////////////////////////////
                case 7:
                {
                    // 发送悬停指令，指令发送成功后开始计时
                    if(hover() == 1)
                    {	fc_condition=4;
                        if(time_dly_cnt_ms < HOVER_STABLE_TIME)
                        {
                            time_dly_cnt_ms += 20;
                        }
                        else
                        {
                            time_dly_cnt_ms = 0;
                            mission_step += 1;  // 进入拐角确认
                        }
                    }
                }
                break;
				
				
/************************************************
 * STEP8：调整中心
 ************************************************/

case 8:
{
    // =========================================
    // 交点位置校准
    // =========================================
    if(g_openmv_cross_flag == 1)
    {
			fc_condition=8;
        // Y方向校准
        if(g_openmv_cross_y > CROSS_Y_DEAD_ZONE)
        {
            Horizontal_Move(
                CROSS_ADJUST_DIST_CM,
                CROSS_ADJUST_SPEED_CMPS,
                0
            );
        }
        else if(g_openmv_cross_y < -CROSS_Y_DEAD_ZONE)
        {
            Horizontal_Move(
                CROSS_ADJUST_DIST_CM,
                CROSS_ADJUST_SPEED_CMPS,
                180
            );
        }

        // X方向校准
        if(g_openmv_cross_x > CROSS_X_DEAD_ZONE)
        {
            Horizontal_Move(
                CROSS_ADJUST_DIST_CM,
                CROSS_ADJUST_SPEED_CMPS,
                90
            );
        }
        else if(g_openmv_cross_x < -CROSS_X_DEAD_ZONE)
        {
            Horizontal_Move(
                CROSS_ADJUST_DIST_CM,
                CROSS_ADJUST_SPEED_CMPS,
                270
            );
        }

        // X、Y都进入死区
        if((g_openmv_cross_x <= CROSS_X_DEAD_ZONE) &&
           (g_openmv_cross_x >= -CROSS_X_DEAD_ZONE) &&
           (g_openmv_cross_y <= CROSS_Y_DEAD_ZONE) &&
           (g_openmv_cross_y >= -CROSS_Y_DEAD_ZONE))
        {
            if(time_dly_cnt_ms < CROSS_STABLE_TIME_MS)
            {
                time_dly_cnt_ms += 20;
            }
            else
            {
                time_dly_cnt_ms = 0;

                // 校准完成，进入计数
                ball_sample_count = 0;
                ball_count_done = 0;

                max_red_count = 0;
                max_green_count = 0;
                max_blue_count = 0;

                mission_step = 9;
            }
        }
        else
        {
            time_dly_cnt_ms = 0;
        }
    }
}
break;

//
/************************************************
 * STEP 9
 *
 * 红、绿、蓝球计数
 * 每次采样间隔20ms
 ************************************************/
case 9:
{
    fc_condition = 7;
    // 每次采样之间延时20ms
    // =========================================
    if(time_dly_cnt_ms < 20)
    {
        time_dly_cnt_ms += 20;
    }
    else
    {
        time_dly_cnt_ms = 0;

        // =========================================
        // 连续采样5帧
        // =========================================
        if(ball_sample_count < 5)
        {
            if(g_openmv_red_count > max_red_count)
                max_red_count = g_openmv_red_count;

            if(g_openmv_green_count > max_green_count)
                max_green_count = g_openmv_green_count;

            if(g_openmv_blue_count > max_blue_count)
                max_blue_count = g_openmv_blue_count;

            ball_sample_count++;
        }
        else
        {
            // 保存当前拐角的球数量
            if(turn_count < 4)
            {
                corner_ball_count[turn_count][0] = max_red_count;
                corner_ball_count[turn_count][1] = max_green_count;
                corner_ball_count[turn_count][2] = max_blue_count;
            }


            // 更新全局当前拐角计数
            g_corner_red_count   += max_red_count;
            g_corner_green_count += max_green_count;
            g_corner_blue_count  += max_blue_count;

            ball_count_done = 1;

            // 计数完成，进入转弯
            mission_step = 10;
        }
    }
}
break;

/************************************************
 * STEP10：执行左转 / 右转
 ************************************************/
case 10:
{
    /**********************************************
     * 只有球数量统计完成后才能执行转弯
     **********************************************/
    if(ball_count_done == 0)
    {
        /* 理论上不会进入这里
         * 如果进入，禁止转弯
         */
        turn_start_flag = 0;
        break;
    }

    /**********************************************
     * 第一次进入STEP10：
     * 发送一次转弯指令
     **********************************************/
    if(turn_start_flag == 0)
    {
        /******************************************
         * 左转
         ******************************************/
        if(turn_direction == 2)
        {
            fc_condition=2;

            if(TurnLeft(TURN_ANGLE, TURN_SPEED) == 1)
            {
                turn_start_flag = 1;
                time_dly_cnt_ms = 0;
            }
        }

        /******************************************
         * 右转
         ******************************************/
        else if(turn_direction == 3)
        {
            fc_condition=5;

            if(TurnRight(TURN_ANGLE, TURN_SPEED) == 1)
            {
                turn_start_flag = 1;
                time_dly_cnt_ms = 0;
            }
        }
    }

    /**********************************************
     * 已经发送过转弯指令
     * 等待转弯执行时间
     **********************************************/
    else
    {
        if(time_dly_cnt_ms < TURN_EXECUTE_TIME)
        {
            time_dly_cnt_ms += 20;
        }
        else
        {
            /****************************************
             * 一次转弯执行结束
             ****************************************/
            time_dly_cnt_ms = 0;

            turn_start_flag = 0;

            /****************************************
             * 转弯次数+1
             ****************************************/
            turn_count++;

            /****************************************
             * 重置OpenMV角度PID
             ****************************************/
            OpenMV_Angle_PID_Reset();

            /****************************************
             * 解锁转弯执行
             ****************************************/
            turn_execute_lock = 0;

            /****************************************
             * 当前角点任务结束
             ****************************************/
            ball_count_done = 0;

            /****************************************
             * 判断是否完成全部转弯
             ****************************************/
            if(turn_count >= TOTAL_TURN_TIMES)
            {
                mission_step = 11;
            }
            else
            {
                mission_step = 6;
            }
        }
    }
}
break;

                //////////////////////////////////////////////////
                // STEP 11
                //
                // 绕正方形一圈完成，保持悬停
                //////////////////////////////////////////////////
                case 11:
                {
					fc_condition=6;
                   // hover(); // 持续悬停，也可替换为 OneKey_Land() 自动降落
					OneKey_Land();
					
					
					
				}

                default:
                {
                }
                break;
            }
        }
        else
        {
            /*
             * 自动任务结束
             */
            mission_step = 0;
            time_dly_cnt_ms = 0;
            turn_count = 0;
			turn_signal_cnt = 0;  // 清零左转信号计数器
            OpenMV_Angle_PID_Reset();
        }
    }
}
