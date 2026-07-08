 /*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"
#include "rls/rls_arm.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#define HWTIMER_DEV_NAME   "timer4"     /* 定时器名称 */
#include <rtdbg.h>

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct chassis_cmd_msg chassis_cmd;
static struct referee_msg referee_fdb;
static struct chassis_fdb_msg chassis_fdb;
static struct ins_msg ins_data;
static struct supercap_fdb_msg cap_fdb;
extern SuperCapRxDataTypeDef supercap_rx;
static publisher_t *pub_chassis;
static subscriber_t *sub_cmd,*sub_ins;
static subscriber_t *sub_referee;

static struct supercap_cmd_msg cap_cmd;
static subscriber_t *sub_cap_cmd;
static subscriber_t *sub_cap_fdb;

static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* -------------------------------- 裁判系统底盘功率相关 ------------------------------- */
//extern robot_status_t robot_status;
//extern ext_power_heat_data_t power_heat_data_t;
/* --------------------------------- 电机控制相关 --------------------------------- */
static pid_obj_t *follow_pid; // 用于底盘跟随云台计算vw
static pid_config_t chassis_follow_config = INIT_PID_CONFIG(CHASSIS_KP_V_FOLLOW, CHASSIS_KI_V_FOLLOW, CHASSIS_KD_V_FOLLOW, CHASSIS_INTEGRAL_V_FOLLOW, CHASSIS_MAX_V_FOLLOW,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
}chassis_controller[4];

static dji_motor_object_t *chassis_motor[4];  // 底盘电机实例
static int16_t motor_ref[4]; // 电机控制期望值

static void chassis_motor_init();
/*定时器初始化*/
static int TIM_Init(void);
/*里程计所需数据*/
static float x_ch, y_ch, w_ch, x_gim, y_gim,vw_ch,vy_ch,vx_ch,vx_gim,vy_gim,x_sin_w,x_cos_w,y_sin_w,y_cos_w;
/* --------------------------------- 底盘运动学解算 --------------------------------- */
/* 根据宏定义选择的底盘类型使能对应的解算函数 */
#ifdef BSP_CHASSIS_OMNI_MODE
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
static void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = omni_calc;
static struct chassis_real_speed_t omni_get_speed(dji_motor_object_t *chassis_motor[4]);
#endif /* BSP_CHASSIS_OMNI_MODE */
#ifdef BSP_CHASSIS_MECANUM_MODE
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = mecanum_calc;
#endif /* BSP_CHASSIS_MECANUM_MODE */

static void absolute_cal(struct chassis_cmd_msg *cmd, float angle);
static struct chassis_real_speed_t
{
    float chassis_vx_ch;
    float chassis_vy_ch;
    float chassis_vw_ch;
    float chassis_vx_gim;
    float chassis_vy_gim;

}chassis_real_speed;

/*================================= 功率控制相关 ================================= */
// #define RLS_POWER_LIMIT//功率限制开关
//#define YAW_CONTROL //yaw轴控制底盘开关
#define K_power 0.10472f//  rpm -> rad/s
#define wheel_ratio    0.05207463310219f  //转子转速转换成轮子转速   1/减速比 ≈ 187/3591 =0.052074
#define K_current 0.001220703125f   //  20 / 16384
#define K_torque  0.3f      //转矩常数 0.3N*M/A
#define CURRENT_TO_TORQUE_RATIO (K_current * K_torque * wheel_ratio)// 转子电流到转子力矩的转换系数，单位：Nm/A   约等于1.9e-5
/* 超电加速时的额外功率 (W) */
#define SUPERCAP_BOOST_EXTRA_POWER    80.0f   // 超电放电时额外提供80W功率
float power_update_timestamp = 0.0f;//检测功率是否更新，用于rls算法

PowerCtrl_Typedef PowerCtrl_Info;
RLS_Info_TypeDef RLS_Power_Info;  // RLS滤波器实例，用于功率模型参数辨识
static float Power_Ctrl_Param[5] = {0.1f, 1.2f, 4.9f, 0.0001f, 500};//RLS拟合初始化参数

float I_cmd[4];//PID计算出来的要发送的电流
uint8_t powerOverloadFlag = 0;  //超功率标志位
float power_max = 60.0f;//便于调试
float Decrease;  // 功率衰减系数
static int16_t motor_target_speed_rad[4];
/* 用于加速度一阶滤波的静态变量 */
// static float chassis_vx_filtered = 0.0f;
// static float chassis_vy_filtered = 0.0f;
// static float last_vx_target = 0.0f;
// static float last_vy_target = 0.0f;
/**
 * @brief 一阶低通滤波器
 * @param current 当前值
 * @param target  目标值
 * @param alpha   滤波系数(0-1),越大响应越快
 * @return 滤波后的值
 */
static float first_order_filter(float current, float target, float alpha)
{
    return current + alpha * (target - current);
}


#ifdef RLS_POWER_LIMIT
/**
 * @brief 功率限制控制主函数
 * @note 基于RLS的自适应功率限制算法，实时监测并限制底盘总功率
 *       功率模型：P = K1*τ² + K2*ω² + τ*ω + K3
 */
void rls_power_limit(uint8_t update_weights)
{
    dji_motor_object_t *motor;       // 电机对象指针,用于获取电机信息
    dji_motor_measure_t measure;     // 电机测量数据结构体（包含转速等信息）
    /* ==================== 功率限制动态调整 ==================== */
    // 根据超电状态调整功率限制
    if (cap_cmd.boost_flag == 1 && cap_fdb.is_ready == 1)
    {
        // 超电放电加速模式：功率限制 = 裁判系统限制 + 超电额外功率
        // 额外功率根据超电剩余能量动态调整
        float energy_ratio = (float)cap_fdb.energy_percent / 100.0f;
        float extra_power = SUPERCAP_BOOST_EXTRA_POWER * energy_ratio;

        PowerCtrl_Info.Power_Max = (referee_fdb.robot_status.chassis_power_limit + extra_power) * 0.9f;  // 90%安全系数
    }
    else
    {
        // 正常模式：功率限制 = 裁判系统限制 * 0.7
        PowerCtrl_Info.Power_Max = referee_fdb.robot_status.chassis_power_limit * 0.8f;
    }
    //PowerCtrl_Info.Power_Max = referee_fdb.robot_status.chassis_power_limit;
/*-------------------------更新RLS拟合部分--------------------------*/
    // 遍历4个底盘电机，计算单电机参数并累积总和
    for (int i = 0; i < 4; i++)
    {
//========================================获取RLS拟合需要的数据====================================//

        motor = chassis_motor[i];               // 获取第i个电机的对象
        measure = motor->measure;               // 获取该电机的实时测量数据
        I_cmd[i] = (float)motor->control(measure) ;     //转子目标输出电流
        PowerCtrl_Info.Measure.Omiga[i] = measure.speed_rad;           //转子实际转速,单位rad/s
        PowerCtrl_Info.Measure.Torque[i] = (float)measure.real_current * CURRENT_TO_TORQUE_RATIO;//转子实际转矩 单位N*M/A

        PowerCtrl_Info.Err[i] = fabsf((float)motor_ref[i] - (float)measure.speed_rad);
        PowerCtrl_Info.Measure.power_useful[i] = PowerCtrl_Info.Measure.Omiga[i] * PowerCtrl_Info.Measure.Torque[i];//禁止加绝对值，会导致疯车
        PowerCtrl_Info.Measure.Omiga_2[i] = powf(PowerCtrl_Info.Measure.Omiga[i], 2.f);
        PowerCtrl_Info.Measure.Torque_2[i] = powf(PowerCtrl_Info.Measure.Torque[i], 2.f);
        //功率模型:P = k1*|w| + k2*τ² + k3 + τ*w
        PowerCtrl_Info.Measure.RLS_Input[i] = (PowerCtrl_Info.Param.K1 * fabsf(PowerCtrl_Info.Measure.Omiga[i]) +
                                               PowerCtrl_Info.Param.K2 * PowerCtrl_Info.Measure.Torque_2[i]);

//========================================获取RLS拟合需要的数据====================================//
    }

    /* ==================== 总和计算与RLS更新 ==================== */

    // 计算四个电机的参数总和
    // 计算总误差绝对值（控制精度指标）
    PowerCtrl_Info.Sum.Err_Sum = PowerCtrl_Info.Err[0] + PowerCtrl_Info.Err[1] +
                                 PowerCtrl_Info.Err[2] + PowerCtrl_Info.Err[3];

    //转子力矩平方总和
    PowerCtrl_Info.Sum.Torque2_Sum = PowerCtrl_Info.Measure.Torque_2[0] + PowerCtrl_Info.Measure.Torque_2[1] +
                                     PowerCtrl_Info.Measure.Torque_2[2] + PowerCtrl_Info.Measure.Torque_2[3];
    //转子角速度平方总和
    PowerCtrl_Info.Sum.Omiga2_Sum = PowerCtrl_Info.Measure.Omiga_2[0] + PowerCtrl_Info.Measure.Omiga_2[1] +
                                    PowerCtrl_Info.Measure.Omiga_2[2] + PowerCtrl_Info.Measure.Omiga_2[3];

    //转子角速度总和
    PowerCtrl_Info.Sum.Omiga_Sum = fabsf(PowerCtrl_Info.Measure.Omiga[0]) + fabsf(PowerCtrl_Info.Measure.Omiga[1]) +
                fabsf(PowerCtrl_Info.Measure.Omiga[2]) + fabsf(PowerCtrl_Info.Measure.Omiga[3]);

    PowerCtrl_Info.Sum.power_useful_Sum = PowerCtrl_Info.Measure.power_useful[0] + PowerCtrl_Info.Measure.power_useful[1] +
                                          PowerCtrl_Info.Measure.power_useful[2] +PowerCtrl_Info.Measure.power_useful[3];

    PowerCtrl_Info.Sum.input_Sum = PowerCtrl_Info.Measure.RLS_Input[0] + PowerCtrl_Info.Measure.RLS_Input[1] +
                                   PowerCtrl_Info.Measure.RLS_Input[2] + PowerCtrl_Info.Measure.RLS_Input[3];
    // 总功率预测 = 各电机功率和 + 固定损耗K3

    RLS_Power_Info.Data.X[0] = PowerCtrl_Info.Sum.Omiga_Sum;
    RLS_Power_Info.Data.X[1] = PowerCtrl_Info.Sum.Torque2_Sum;

    // RLS期望输出：模型预测功率
    RLS_Power_Info.Data.U[0] = PowerCtrl_Info.Sum.input_Sum ;
    // RLS实际输出：功率计测量的底盘实际功率 + 偏移量3W
    RLS_Power_Info.Data.Y[0] = cap_fdb.chassis_power - PowerCtrl_Info.Sum.power_useful_Sum - PowerCtrl_Info.Param.K3 + 5 ;//加3w待定

    if (PowerCtrl_Info.Sum.Omiga_Sum > 5.0f && update_weights)//防止静止时也拟合,导致拟合发散
    {
        // 更新RLS滤波器权重参数
        RLS_Update(&RLS_Power_Info);
        PowerCtrl_Info.Param.K1 = fmaxf(RLS_Power_Info.Data.W[0], 1e-3f);
        PowerCtrl_Info.Param.K2 = fmaxf(fminf(RLS_Power_Info.Data.W[1],20.0f), 1e-3f);
        // 使用新参数重新计算总功率预测
        PowerCtrl_Info.Sum.Power_Sum =PowerCtrl_Info.Param.K1 * PowerCtrl_Info.Sum.Omiga_Sum +
                                      PowerCtrl_Info.Param.K2 * PowerCtrl_Info.Sum.Torque2_Sum + PowerCtrl_Info.Param.K3 + PowerCtrl_Info.Sum.power_useful_Sum;
    }

/*-------------------------更新RLS拟合部分--------------------------*/


/*-----------------------------功率分配部分-----------------------*/
    powerOverloadFlag = 0;  //清除超功率标志位
    if (PowerCtrl_Info.Sum.Power_Sum > PowerCtrl_Info.Power_Max)
    {
        powerOverloadFlag = 1;  //开启超功率标志位
        // 计算功率分配因子K（基于误差大小的自适应权重）
        if (PowerCtrl_Info.Sum.Err_Sum > PowerCtrl_Info.Param.Err_Upper)
            PowerCtrl_Info.K = 1;  // 误差大，完全按误差分配
        else if (PowerCtrl_Info.Sum.Err_Sum < PowerCtrl_Info.Param.Err_Lower)
            PowerCtrl_Info.K = 0;  // 误差小，完全按功率分配
        else
            // 误差中等,线性插值分配
            PowerCtrl_Info.K =
                    fmaxf(0.0f,
                          fminf(1.0f,
                                (PowerCtrl_Info.Sum.Err_Sum - PowerCtrl_Info.Param.Err_Lower) /
                                (PowerCtrl_Info.Param.Err_Upper - PowerCtrl_Info.Param.Err_Lower)
                          )
                    );

        // 计算每个电机的功率分配权重（港科大论文的功率分配方法）
        for (int i = 0; i < 4; i++)
        {
            // 1. 计算每个电机的权重（0-1之间）
            float error_weight = PowerCtrl_Info.Power_Max * (PowerCtrl_Info.Err[i] / PowerCtrl_Info.Sum.Err_Sum);
            float power_weight = PowerCtrl_Info.Power_Max * ((PowerCtrl_Info.Measure.RLS_Input[i] +
                                                              PowerCtrl_Info.Measure.power_useful[i] +
                                                              PowerCtrl_Info.Param.K3 * 0.25f) / PowerCtrl_Info.Sum.Power_Sum);
            PowerCtrl_Info.Power_Limit[i] = PowerCtrl_Info.K * error_weight + (1.0f - PowerCtrl_Info.K) * power_weight;
        }

        // 计算功率衰减系数：功率限制/实际功率，限制在[0,1]范围
        Decrease = PowerCtrl_Info.Power_Max / PowerCtrl_Info.Sum.Power_Sum;
        VAL_LIMIT(Decrease, 0, 1);

        // 对每个电机进行功率限制计算
        for (int i = 0; i < 4; i++)
        {
            /* 单个电机求解功率方程：P_limit = K1*|ω| + K2*τ² + τ*ω + K3/4
               整理为二次方程：K1*|ω| + ω*τ + (K2*τ² + K3/4 - P_limit) = 0
               标准形式：A*τ² + B*τ + C = 0
            */
            PowerCtrl_Info.A = PowerCtrl_Info.Param.K2;  // 二次项系数
            PowerCtrl_Info.B = (float)motor_ref[i];  // 一次项系数（角速度）
            PowerCtrl_Info.C = fabsf((float)motor_ref[i]) * PowerCtrl_Info.Param.K1 +
                               PowerCtrl_Info.Param.K3 * 0.25f - PowerCtrl_Info.Power_Limit[i];  // 常数项

            // 计算判别式Δ = B² - 4AC
            PowerCtrl_Info.Delta = powf(PowerCtrl_Info.B, 2.f) - 4 * PowerCtrl_Info.A * PowerCtrl_Info.C;

            // 检查判别式是否有效
            if (isnan(PowerCtrl_Info.Delta) == 1 || isinf(PowerCtrl_Info.Delta) == 1)
                PowerCtrl_Info.Delta = 0;  // 无效值处理

            // 根据判别式大小求解转矩
            if (PowerCtrl_Info.Delta >= 0)  // 有实数解
            {
                PowerCtrl_Info.Sqrt = sqrtf(PowerCtrl_Info.Delta);

                if (I_cmd[i] >= 0.0f)  // 正转情况
                {
                    // 取正根解
                    PowerCtrl_Info.Torque[i] = (-PowerCtrl_Info.B + PowerCtrl_Info.Sqrt) / (2 * PowerCtrl_Info.A);
                    // 转矩转电流，并应用功率衰减
                    PowerCtrl_Info.Output[i] = (PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO * Decrease);
                }
                else  // 反转情况
                {
                    // 取负根解
                    PowerCtrl_Info.Torque[i] = (-PowerCtrl_Info.B - PowerCtrl_Info.Sqrt) / (2 * PowerCtrl_Info.A);
                    PowerCtrl_Info.Output[i] = ((PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO) * Decrease);
                }
            }
            else  // 无实数解，使用近似解
            {
                if (I_cmd[i] >= 0)
                {
                    // 使用顶点近似：τ = -B/2A
                    PowerCtrl_Info.Torque[i] = (-PowerCtrl_Info.B) / (2.0f * PowerCtrl_Info.A);
                    PowerCtrl_Info.Output[i] = ((PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO) * Decrease);
                }
                else
                {
                    PowerCtrl_Info.Torque[i] = (PowerCtrl_Info.B) / (2.0f * PowerCtrl_Info.A);
                    PowerCtrl_Info.Output[i] = ((PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO) * Decrease);
                }
            }
            VAL_LIMIT(PowerCtrl_Info.Output[i], -8000, 8000);//限幅防止跑飞
        }
    }

}
/**
 * @brief 功率控制系统初始化
 * @param PowerCtrl_Info 功率控制信息结构体指针
 * @param Lamda RLS遗忘因子（0-1，值越大记忆越长）
 * @param P RLS协方差矩阵初始值
 * @param PowerCtrl_Param 功率控制参数数组
 * @note 初始化RLS滤波器和功率控制参数
 */
void PowerCtrl_Init(PowerCtrl_Typedef *Power_Info, float Lamda, float P, float PowerCtrlParam[5])
{
    // 初始化RLS滤波器：2个输入(转矩平方和,角速度平方和)，1个输出(功率)
    RLS_Init(&RLS_Power_Info, 2, 1, Lamda, P);

    // 设置功率模型参数：K1-转速损耗系数，K2-转矩损耗系数，K3-固定损耗
    Power_Info->Param.K1 = PowerCtrlParam[0];
    Power_Info->Param.K2 = PowerCtrlParam[1];
    Power_Info->Param.K3 = PowerCtrlParam[2];

    // 设置误差阈值：用于功率分配权重计算
    Power_Info->Param.Err_Lower = PowerCtrlParam[3];  // 误差下限
    Power_Info->Param.Err_Upper = PowerCtrlParam[4];  // 误差上限

    // 初始化RLS权重参数
    RLS_Power_Info.Data.W[0] = Power_Info->Param.K1;
    RLS_Power_Info.Data.W[1] = Power_Info->Param.K2;

}

#endif

/* --------------------------------- 底盘线程入口 --------------------------------- */
static float cmd_dt;

void chassis_thread_entry(void *argument)
{
    static float cmd_start;

    chassis_pub_init();
    chassis_sub_init();
    chassis_motor_init();
    TIM_Init();

#ifdef RLS_POWER_LIMIT
    PowerCtrl_Init(&PowerCtrl_Info,0.99999f,1e-5f,Power_Ctrl_Param);
#endif

    LOG_I("Chassis Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        /* 计算实际速度 */
        omni_get_speed(chassis_motor);
        /* 更新该线程所有的订阅者 */
        chassis_sub_pull();

        for (uint8_t i = 0; i < 4; i++)
        {
            dji_motor_enable(chassis_motor[i]);
        }

        // /* ============ 【新增】Boost模式动态参数调整 ============ */
        // float current_limit = CHASSIS_NORMAL_CURRENT_LIMIT;   // 默认正常电流限制
        // float acc_limit = CHASSIS_NORMAL_ACC_LIMIT;           // 默认正常加速度
        //
        // // 判断是否启用Boost模式
        // if (cap_cmd.boost_flag == 1 &&           // X键按下
        //     cap_fdb.is_ready == 1)               // 超电就绪
        // {
        //     // Boost模式 - 提高电流和加速度限制
        //     current_limit = CHASSIS_BOOST_CURRENT_LIMIT;
        //     acc_limit = CHASSIS_BOOST_ACC_LIMIT;
        // }
        //
        // /* 应用加速度限制 - 这是触发超电的关键 */
        // float max_delta_v = acc_limit * 0.001f; // 1ms周期,单位mm/s²
        //
        // // 计算速度变化量
        // float delta_vx = chassis_cmd.vx - last_vx_target;
        // float delta_vy = chassis_cmd.vy - last_vy_target;
        //
        // // 限制加速度
        // VAL_LIMIT(delta_vx, -max_delta_v, max_delta_v);
        // VAL_LIMIT(delta_vy, -max_delta_v, max_delta_v);
        //
        // // 更新目标速度
        // float vx_target = last_vx_target + delta_vx;
        // float vy_target = last_vy_target + delta_vy;
        //
        // // 保存本次目标值
        // last_vx_target = vx_target;
        // last_vy_target = vy_target;

        // // 一阶滤波平滑过渡
        //chassis_vx_filtered = first_order_filter(chassis_vx_filtered, vx_target,
                                                  //CHASSIS_ACC_FILTER_ALPHA);
        //chassis_vy_filtered = first_order_filter(chassis_vy_filtered, vy_target,
                                                  //CHASSIS_ACC_FILTER_ALPHA);

        // // 使用滤波后的速度替换原始指令
        //chassis_cmd.vx = chassis_vx_filtered;
        //chassis_cmd.vy = chassis_vy_filtered;
        /* ============ Boost模式处理结束 ============ */

        switch (chassis_cmd.ctrl_mode)
        {
        case CHASSIS_RELAX:
            for (uint8_t i = 0; i < 4; i++)
            {
                dji_motor_relax(chassis_motor[i]);
            }
            break;
        case CHASSIS_FOLLOW_GIMBAL:
            chassis_cmd.vw = pid_calculate(follow_pid, chassis_cmd.offset_angle, SIDEWAYS_ANGLE);
            /* 底盘运动学解算 */
            absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
            chassis_calc_moto_speed(&chassis_cmd, motor_ref);
            break;
        case CHASSIS_SPIN:
            absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
            chassis_calc_moto_speed(&chassis_cmd, motor_ref);
            break;
        case CHASSIS_OPEN_LOOP:
            chassis_calc_moto_speed(&chassis_cmd, motor_ref);
            break;
        case CHASSIS_STOP:
            rt_memset(motor_ref, 0, sizeof(motor_ref));
            break;
        case CHASSIS_FLY:
            break;
        case CHASSIS_AUTO:
            break;
        default:
            for (uint8_t i = 0; i < 4; i++)
            {
                dji_motor_relax(chassis_motor[i]);
            }
            break;
        }

#ifdef RLS_POWER_LIMIT
        static float last_power_timestamp = 0.0f;  // 记录上次RLS更新权重时的时间戳
        static uint8_t should_update_weights = 0;
        should_update_weights = 0;
        // 通过比较时间戳判断功率数据是否更新
        if (power_update_timestamp != last_power_timestamp)
        {
            should_update_weights = 1;
            last_power_timestamp = power_update_timestamp;
        }
        // 每个周期都执行,但只在功率数据更新和运动时才更新权重系数
        rls_power_limit(should_update_weights);
#endif
        dji_motor_control();

        /* 更新发布该线程的msg */
        chassis_pub_push();

        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
            LOG_E("Chassis Task is being DELAY! dt = [%f]", &cmd_dt);

        rt_thread_delay(1);
    }
}

/**
 * @brief chassis 线程中所有发布者初始化
 */
static void chassis_pub_init(void)
{
    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    sub_referee= sub_register("referee_fdb", sizeof(struct referee_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_cap_cmd = sub_register("cap_cmd", sizeof(struct supercap_cmd_msg));
    sub_cap_fdb = sub_register("cap_fdb", sizeof(struct supercap_fdb_msg));
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    pub_push_msg(pub_chassis,&chassis_fdb);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    sub_get_msg(sub_cmd, &chassis_cmd);
    sub_get_msg(sub_referee, &referee_fdb);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_cap_cmd, &cap_cmd);
    sub_get_msg(sub_cap_fdb, &cap_fdb);

}

/* --------------------------------- 电机控制相关 --------------------------------- */
#define CURRENT_POWER_LIMIT_RATE 80
static rt_int16_t motor_control_0(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
    /*传参给局部变量防止被更改抽风*/
    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
    /*底盘功率限制防止buffer溢出*/
    if(chassis_power_limit>=120)
    {
        chassis_power_limit=120;
    }
    if(referee_fdb.power_heat_data.buffer_energy<20)
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
    }
    else
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(rt_int16_t) pid_calculate(chassis_controller[0].speed_pid, measure.speed_rpm, motor_ref[0]);
    // VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

static rt_int16_t motor_control_1(dji_motor_measure_t measure)
{

    static rt_int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
    /*传参给局部变量防止被更改抽风*/
    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
    /*底盘功率限制防止buffer溢出*/
    if(chassis_power_limit>=120)
    {
        chassis_power_limit=120;
    }
    if(referee_fdb.power_heat_data.buffer_energy<20)
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
    }
    else
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(rt_int16_t) pid_calculate(chassis_controller[1].speed_pid, measure.speed_rpm, motor_ref[1]);
    // VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

static rt_int16_t motor_control_2(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
    /*传参给局部变量防止被更改抽风*/
    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
    /*底盘功率限制防止buffer溢出*/
    if(chassis_power_limit>=120)
    {
        chassis_power_limit=120;
    }
    if(referee_fdb.power_heat_data.buffer_energy<20)
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
    }
    else
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(rt_int16_t) pid_calculate(chassis_controller[2].speed_pid, measure.speed_rpm, motor_ref[2]);
    // VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

static rt_int16_t motor_control_3(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    static int16_t chassis_max_current=0;
    static int16_t chassis_power_limit=0;
    /*传参给局部变量防止被更改抽风*/
    chassis_power_limit=(int16_t)referee_fdb.robot_status.chassis_power_limit;
    /*底盘功率限制防止buffer溢出*/
    if(chassis_power_limit>=120)
    {
        chassis_power_limit=120;
    }
    if(referee_fdb.power_heat_data.buffer_energy<20)
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE*(referee_fdb.power_heat_data.buffer_energy/50);
    }
    else
    {
        chassis_max_current=chassis_power_limit*CURRENT_POWER_LIMIT_RATE;
    }
    if (chassis_power_limit==0)
    {
        chassis_max_current=8000;
    }
    set =(rt_int16_t) pid_calculate(chassis_controller[3].speed_pid, measure.speed_rpm, motor_ref[3]);
    // VAL_LIMIT(set , -chassis_max_current, chassis_max_current);
    return set;
}

/* 底盘每个电机对应的控制函数 */
static void *motor_control[4] =
        {
                motor_control_0,
                motor_control_1,
                motor_control_2,
                motor_control_3,
        };

// TODO：将参数都放到配置文件中，通过宏定义进行替换
motor_config_t chassis_motor_config[4] =
        {
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x201,
                        .controller = &chassis_controller[0],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x202,
                        .controller = &chassis_controller[1],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x203,
                        .controller = &chassis_controller[2],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x204,
                        .controller = &chassis_controller[3],
                }
        };

/**
 * @brief 注册底盘电机及其控制器初始化
 */
static void chassis_motor_init()
{
    pid_config_t chassis_speed_config = INIT_PID_CONFIG(CHASSIS_KP_V_MOTOR, CHASSIS_KI_V_MOTOR, CHASSIS_KD_V_MOTOR, CHASSIS_INTEGRAL_V_MOTOR, CHASSIS_MAX_V_MOTOR,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_controller[i].speed_pid = pid_register(&chassis_speed_config);
        chassis_motor[i] = dji_motor_register(&chassis_motor_config[i], motor_control[i]);

        chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
        chassis_cmd.last_mode = CHASSIS_OPEN_LOOP;
    }
    follow_pid = pid_register(&chassis_follow_config);
}

/* --------------------------------- 底盘解算控制 --------------------------------- */
#ifdef BSP_CHASSIS_OMNI_MODE
/**
 * @brief 全向轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    int16_t wheel_rpm[4];
    int16_t wheel_rad[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //rad/s

    wheel_rpm[0] = ( 0.7071*cmd->vx + 0.7071*cmd->vy + cmd->vw * LENGTH_RADIUS) * wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = ( 0.7071*cmd->vx - 0.7071*cmd->vy + cmd->vw * LENGTH_RADIUS) * wheel_rpm_ratio;//forward
    wheel_rpm[2] = (0.7071*-cmd->vx - 0.7071*cmd->vy + cmd->vw * LENGTH_RADIUS) * wheel_rpm_ratio;//right
    wheel_rpm[3] = (0.7071*-cmd->vx + 0.7071*cmd->vy + cmd->vw * LENGTH_RADIUS) * wheel_rpm_ratio;//back

//    wheel_rad[0] = wheel_rpm[0] * 0.10472f;
//    wheel_rad[1] = wheel_rpm[1] * 0.10472f;
//    wheel_rad[2] = wheel_rpm[2] * 0.10472f;
//    wheel_rad[3] = wheel_rpm[3] * 0.10472f;
    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}

/**
 * @brief 全向轮底盘运动逆解算求解实际速度(x,y,w是相对于底盘的x，y，w的速度)
 *
 * @param TODO:具体数值正负待定，由测试得正确结果
 * @param
 */

static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    x_ch = vx_ch *0.001f +x_ch;
    y_ch = vy_ch *0.001f +y_ch;
    w_ch = vw_ch * 0.001f * RADIAN_COEF +w_ch;
    x_gim = vx_gim *0.001f +x_gim;
    y_gim = vy_gim *0.001f +y_gim;
    x_cos_w = vx_gim * cosf(ins_data.yaw_total_angle/RADIAN_COEF)*0.001f + x_cos_w;
    x_sin_w = vy_gim * sinf(ins_data.yaw_total_angle/RADIAN_COEF)*0.001f + x_sin_w;
    y_cos_w = vy_gim * cosf(ins_data.yaw_total_angle/RADIAN_COEF)*0.001f + y_cos_w;
    y_sin_w = vx_gim * sinf(ins_data.yaw_total_angle/RADIAN_COEF)*0.001f + y_sin_w;
    chassis_fdb.x_pos_gim=x_cos_w + x_sin_w;
    chassis_fdb.y_pos_gim=y_cos_w - y_sin_w;

    return 0;
}

int TIM_Init(void)
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;      /* 定时器超时值 */
    rt_device_t hw_dev = RT_NULL;   /* 定时器设备句柄 */
    rt_hwtimer_mode_t mode;         /* 定时器模式 */
    rt_uint32_t freq = 10000;               /* 计数频率 */

    /* 查找定时器设备 */
    hw_dev = rt_device_find("timer4" );
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    timeout_s.sec = 0;      /* 秒 */
    timeout_s.usec = 1000;     /* 微秒 */
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }
}

static struct chassis_real_speed_t omni_get_speed(dji_motor_object_t *chassis_motor[4])//里程计计算函数。
{

    float angle_hd = -(chassis_cmd.offset_angle/RADIAN_COEF);
    float wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    for (int i = 0; i < 4; ++i)
    {
        wheel_rpm[i]=chassis_motor[i]->measure.speed_rad;
    }
    struct chassis_real_speed_t real_speed;
    vw_ch =real_speed.chassis_vw_ch = (wheel_rpm[0] + wheel_rpm[1] + wheel_rpm[2] + wheel_rpm[3]) / (4.0f * wheel_rpm_ratio * LENGTH_RADIUS);
    vy_ch =real_speed.chassis_vy_ch = (wheel_rpm[0] + wheel_rpm[3] - wheel_rpm[1] - wheel_rpm[2]) / (4.0f * 0.7071f * wheel_rpm_ratio);
    vx_ch =real_speed.chassis_vx_ch = (wheel_rpm[0] + wheel_rpm[1] - wheel_rpm[2] - wheel_rpm[3]) / (4.0f * 0.7071f * wheel_rpm_ratio) ;
    vx_gim =real_speed.chassis_vx_gim =  real_speed.chassis_vx_ch* cos(angle_hd) - real_speed.chassis_vy_ch* sin(angle_hd);
    vy_gim =real_speed.chassis_vy_gim =  real_speed.chassis_vx_ch* sin(angle_hd) + real_speed.chassis_vy_ch* cos(angle_hd);

    return real_speed;
}
#endif /* BSP_CHASSIS_OMNI_MODE */

#ifdef BSP_CHASSIS_MECANUM_MODE
/**
 * @brief 麦克纳姆轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    static float rotate_ratio_f = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float rotate_ratio_b = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    float max = 0;

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = ( cmd->vx - cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = ( cmd->vx + cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (-cmd->vx + cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (-cmd->vx - cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;

    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));
}
#endif /* BSP_CHASSIS_MECANUM_MODE */

/**
  * @brief  将云台坐标转换为底盘坐标
  * @param  cmd 底盘指令值，使用其中的速度
  * @param  angle 云台相对于底盘的角度
  */
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle)
{
    float angle_hd = -(angle/RADIAN_COEF);
    float vx = cmd->vx;
    float vy = cmd->vy;

    //保证底盘是相对摄像头做移动
    cmd->vx = vx * cos(angle_hd) + vy * sin(angle_hd);
    cmd->vy = -vx * sin(angle_hd) + vy * cos(angle_hd);
}
