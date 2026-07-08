#ifndef SUPERCAP_TASK_H
#define SUPERCAP_TASK_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>

/* ================================ 宏定义 ================================ */

#define SUPERCAP_OFFLINE_TIME    300    // 离线判定时间(ms)
#define SUPERCAP_TASK_PERIOD     1      // 任务周期(ms)，200Hz发送频率    ///////我改2
#define CAN_ID_BUS_TO_SUPERCAP   0x221  // 电控->超电
#define CAN_ID_SUPERCAP_TO_BUS   0x220  // 超电->电控

/* ================================ 枚举定义 ================================ */

/* 超电就绪状态 */
typedef enum
{
    SUPERCAP_READY_NO  = 0,   // 不可用
    SUPERCAP_READY_YES = 1,   // 可用
} SuperCapReadyTypeDef;

/* 超电工作状态 */
typedef enum
{
    SUPERCAP_STATE_DISCHARGING = 0,   // 放电
    SUPERCAP_STATE_CHARGING    = 1,   // 充电
} SuperCapStateTypeDef;

/* 超电使能状态 */
typedef enum
{
    SUPERCAP_STATE_DISENABLE = 0,   // 失能
    SUPERCAP_STATE_ENABLE    = 1,   // 使能
} SuperCapEnableTypeDef;
/* ================================ 结构体定义 ================================ */

/* 发送数据结构 */
typedef struct
{
    uint8_t Enable;         // 使能
    uint8_t Charge;         // 充放电
    uint8_t PowerLimit;     // 功率限制
    uint8_t ChargePower;    // 充电功率
} SuperCapTxDataTypeDef;

/* 接收数据结构 */
typedef struct
{
    SuperCapReadyTypeDef SuperCapReady;       // 就绪状态
    SuperCapStateTypeDef SuperCapState;       // 工作状态
    uint8_t              SuperCapEnergy;      // 能量百分比 0-100
    uint8_t                ChassisPower;        // 底盘功率
    uint8_t              VoltageBat;          // 电池电压
    uint8_t              DebugOut_BatPower;   // 调试-电池功率
    int8_t               DebugOut_SuperCapPower; // 调试-超电功率
} SuperCapRxDataTypeDef;

/* 控制指令消息 */
struct supercap_cmd_msg
{
    uint8_t  enable_cmd;          // 使能命令: 0=失能, 1=使能
    uint8_t  charge_cmd;          // 充放电命令: 0=放电, 1=充电
    uint8_t  charge_power;        // 充电功率 (W)
    uint8_t  discharge_power;     // 放电功率 (W) - 发送给超电板
    uint8_t  power_limit;         // 底盘功率限制 (W)
    uint8_t  boost_flag;          // 加速标志
    float    motor_need_power;    // 电机需要的总功率 (W)
};

/* 反馈消息 */
struct supercap_fdb_msg
{
    uint8_t  is_online;           // 超电是否在线
    uint8_t  is_ready;            // 超电是否可用
    uint8_t  is_charging;          // 超电是否充电
    uint8_t  energy_percent;      // 能量百分比 (0-100)
    uint8_t  tick;                  //用于计数连续超过多少次功限
    float    input_voltage;       // 输入电压 (V)
    float    cap_voltage;         // 电容电压 (V)
    float    input_current;       // 输入电流 (A)
    float    output_power;        // 超电实际输出功率 (W)
    float    chassis_power;       // 底盘实际功率 (W)
    float    avg_power;           // 超限时的平均功率

};

/* ================================ 函数声明 ================================ */

/**
 * @brief 超级电容任务入口
 */
void supercap_thread_entry(void *argument);

/**
 * @brief CAN接收回调
 */
void supercap_can_rx_callback(rt_device_t dev, uint32_t id, uint8_t *data);

/**
 * @brief 兼容接口 - 接收处理
 */
void supercap_rx_handler(uint8_t *data);

#endif /* SUPERCAP_TASK_H */