#include "ui.h"

#include <stdio.h>

#include "priority_ui.h"
#include "chassis_task.h"
#include "dji_motor.h"

#define AIM_X  956
#define AIM_Y  395
void rotate(uint16_t *x, uint16_t *y, float mid_x, float mid_y, float angle);
void rotate_arc(ui_config_t *config, float mid_x, float mid_y, float angle);
void Spin_Rp_Ui(void);
/**
 *
 */
ui_info_t dynamic_ui_info [DYNAMIC_UI_NUM] =
        {
        [Frc] = {
                /*不变配置*/
                .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = CHAR, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = FUCHSIA, // 颜色
                .ui_config.size = 35, // 字体大小
                .ui_config.width = 3, // 线条宽度
                .ui_config.start_x = 430, // 起点 x 坐标
                .ui_config.start_y = 850, // 起点 y 坐标
                .ui_config.text = "Frc                             ",
        },
        [Rot] = {
                /*不变配置*/
                .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = CHAR, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = FUCHSIA, // 颜色
                .ui_config.size = 35, // 字体大小
                .ui_config.width = 3, // 线条宽度
                .ui_config.start_x = 430, // 起点 x 坐标
                .ui_config.start_y = 775, // 起点 y 坐标
                .ui_config.text = "Rot                            ",
        },
        [Aim] = {
                /*不变配置*/
                .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = CHAR, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = FUCHSIA, // 颜色
                .ui_config.size = 35, // 字体大小
                .ui_config.width = 3, // 线条宽度
                .ui_config.start_x = 430, // 起点 x 坐标
                .ui_config.start_y = 700, // 起点 y 坐标
                .ui_config.text = "Aim                                ",
        },

        [Heat_Indicator] = {
                /*不变配置*/
                .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = CHAR, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 5, // 图层数，0~9
                .ui_config.color = GREEN, // 颜色
                .ui_config.size = 35, // 字体大小
                .ui_config.width = 3, // 线条宽度
                .ui_config.start_x = 1320, // 起点 x 坐标
                .ui_config.start_y = 720, // 起点 y 坐标
                .ui_config.text = "HEAT:                                ",
        },

        [Aim_Frame] = {
                /*不变配置*/
                .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = CIRCLE, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = RED_BLUE, // 颜色
                .ui_config.width = 5, // 线条宽度
                .ui_config.start_x = 960, // 起点 x 坐标
                .ui_config.start_y = 540, // 起点 y 坐标
                .ui_config.radius = 250,
        },
        [Carline_L] = {
                /*不变配置*/
                .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = LINE, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = ORANGE, // 颜色
                .ui_config.width = 5, // 线条宽度
                .ui_config.start_x = 529, // 起点 x 坐标
                .ui_config.start_y = 102, // 起点 y 坐标
                .ui_config.end_x = 783,
                .ui_config.end_y = 445,
        },
        [Carline_R] = {
                /*不变配置*/
                .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = LINE, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = ORANGE, // 颜色
                .ui_config.width = 5, // 线条宽度
                .ui_config.start_x = 1391, // 起点 x 坐标
                .ui_config.start_y = 102, // 起点 y 坐标
                .ui_config.end_x = 1137,
                .ui_config.end_y = 445,
        },
        [Cross_H] = {
                /*不变配置*/
                .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = LINE, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = CYAN_BLUE, // 颜色
                .ui_config.width = 2, // 线条宽度
                .ui_config.start_x = 935, // 起点 x 坐标
                .ui_config.start_y = 540, // 起点 y 坐标
                .ui_config.end_x = 985,
                .ui_config.end_y = 540,
        },
        [Cross_V] = {
                /*不变配置*/
                .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
                .ui_config.ui_type = LINE, // UI内容类型
                /*可变配置*/
                .ui_config.operate_type = MODIFY, // 操作类型
                .ui_config.layer = 3, // 图层数，0~9
                .ui_config.color = CYAN_BLUE, // 颜色
                .ui_config.width = 2, // 线条宽度
                .ui_config.start_x = 960, // 起点 x 坐标
                .ui_config.start_y = 570, // 起点 y 坐标
                .ui_config.end_x = 960,
                .ui_config.end_y = 180,
        },

    ///测试ui  无意义///
//     [rectangle1] = {
//             /*不变配置*/
//             .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//             .ui_config.ui_type = RECTANGEL, // UI内容类型
//             /*可变配置*/
//             .ui_config.operate_type = MODIFY, // 操作类型
//             .ui_config.layer = 5, // 图层数，0~9
//             .ui_config.color = YELLOW, // 颜色
//             .ui_config.size = 35, // 字体大小
//             .ui_config.width = 3, // 线条宽度
//             .ui_config.start_x =0, // 起点 x 坐标
//             .ui_config.start_y = 0, // 起点 y 坐标
//             .ui_config.end_x = 100, // 起点 x 坐标
//             .ui_config.end_y = 100, // 起点 y 坐标
//     },
//
//     [rectangle2] = {
//             /*不变配置*/
//             .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//             .ui_config.ui_type = RECTANGEL, // UI内容类型
//             /*可变配置*/
//             .ui_config.operate_type = MODIFY, // 操作类型
//             .ui_config.layer = 5, // 图层数，0~9
//             .ui_config.color = YELLOW, // 颜色
//             .ui_config.size = 35, // 字体大小
//             .ui_config.width = 3, // 线条宽度
//             .ui_config.start_x =150, // 起点 x 坐标
//             .ui_config.start_y = 150, // 起点 y 坐标
//             .ui_config.end_x = 250, // 起点 x 坐标
//             .ui_config.end_y = 250, // 起点 y 坐标
//     },
//
//     [rectangle3] = {
//             /*不变配置*/
//             .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//             .ui_config.ui_type = RECTANGEL, // UI内容类型
//             /*可变配置*/
//             .ui_config.operate_type = MODIFY, // 操作类型
//             .ui_config.layer = 5, // 图层数，0~9
//             .ui_config.color = YELLOW, // 颜色
//             .ui_config.size = 35, // 字体大小
//             .ui_config.width = 3, // 线条宽度
//             .ui_config.start_x =300, // 起点 x 坐标
//             .ui_config.start_y = 300, // 起点 y 坐标
//             .ui_config.end_x = 400, // 起点 x 坐标
//             .ui_config.end_y = 400, // 起点 y 坐标
//     },
//
//     [rectangle4] = {
//             /*不变配置*/
//             .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//             .ui_config.ui_type = RECTANGEL, // UI内容类型
//             /*可变配置*/
//             .ui_config.operate_type = MODIFY, // 操作类型
//             .ui_config.layer = 5, // 图层数，0~9
//             .ui_config.color = YELLOW, // 颜色
//             .ui_config.size = 35, // 字体大小
//             .ui_config.width = 3, // 线条宽度
//             .ui_config.start_x =450, // 起点 x 坐标
//             .ui_config.start_y = 450, // 起点 y 坐标
//             .ui_config.end_x = 550, // 起点 x 坐标
//             .ui_config.end_y = 550, // 起点 y 坐标
//     },
//     [rectangle5] = {
//             /*不变配置*/
//             .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//             .ui_config.ui_type = RECTANGEL, // UI内容类型
//             /*可变配置*/
//             .ui_config.operate_type = MODIFY, // 操作类型
//             .ui_config.layer = 5, // 图层数，0~9
//             .ui_config.color = YELLOW, // 颜色
//             .ui_config.size = 35, // 字体大小
//             .ui_config.width = 3, // 线条宽度
//             .ui_config.start_x =600, // 起点 x 坐标
//             .ui_config.start_y = 600, // 起点 y 坐标
//             .ui_config.end_x = 700, // 起点 x 坐标
//             .ui_config.end_y = 700, // 起点 y 坐标
//     },
//
//     [rectangle6] = {
//         /*不变配置*/
//         .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//         .ui_config.ui_type = RECTANGEL, // UI内容类型
//         /*可变配置*/
//         .ui_config.operate_type = MODIFY, // 操作类型
//         .ui_config.layer = 5, // 图层数，0~9
//         .ui_config.color = YELLOW, // 颜色
//         .ui_config.size = 35, // 字体大小
//         .ui_config.width = 3, // 线条宽度
//         .ui_config.start_x =750, // 起点 x 坐标
//         .ui_config.start_y = 750, // 起点 y 坐标
//         .ui_config.end_x = 850, // 起点 x 坐标
//         .ui_config.end_y = 850, // 起点 y 坐标
// },
//
//         [rectangle7] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =750, // 起点 x 坐标
//                 .ui_config.start_y = 750, // 起点 y 坐标
//                 .ui_config.end_x = 850, // 起点 x 坐标
//                 .ui_config.end_y = 850, // 起点 y 坐标
//         },
//
//         [rectangle8] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
//
//         [rectangle9] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
//         [rectangle10] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
//         [rectangle11] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
//         [rectangle12] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
//         [rectangle13] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
//
//         [rectangle14] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
//
//         [rectangle15] = {
//                 /*不变配置*/
//                 .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//                 .ui_config.ui_type = RECTANGEL, // UI内容类型
//                 /*可变配置*/
//                 .ui_config.operate_type = MODIFY, // 操作类型
//                 .ui_config.layer = 5, // 图层数，0~9
//                 .ui_config.color = YELLOW, // 颜色
//                 .ui_config.size = 35, // 字体大小
//                 .ui_config.width = 3, // 线条宽度
//                 .ui_config.start_x =900, // 起点 x 坐标
//                 .ui_config.start_y =900, // 起点 y 坐标
//                 .ui_config.end_x = 1000, // 起点 x 坐标
//                 .ui_config.end_y = 1000, // 起点 y 坐标
//         },
        };



ui_info_t const_ui_info [0] =
        {

        };


/**
  @brief: 初始化UI链表 main.c中调用
 */
void My_Ui_Init(void)
{
    Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
}

extern uint8_t send_test;

extern uint8_t send_test;
static char s[5] = "Heat:";
static char t[1] = "/";

void Ui_Info_Update(struct gimbal_cmd_msg gim_cmd,struct chassis_cmd_msg chassis_cmd,struct shoot_cmd_msg shoot_cmd,struct referee_msg referee)
{
    if (send_test != 0)
    {
        return;
    }

    if (shoot_cmd.friction_status == 1)
    {
        dynamic_ui_info[Frc].ui_config.color = GREEN;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[Frc]);
    }
    else
    {
        dynamic_ui_info[Frc].ui_config.color = FUCHSIA;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[Frc]);
    }

    if (chassis_cmd.ctrl_mode == CHASSIS_SPIN)
    {
        dynamic_ui_info[Rot].ui_config.color = GREEN;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[Rot]);
    }
    else
    {
        dynamic_ui_info[Rot].ui_config.color = FUCHSIA;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[Rot]);
    }

    if (gim_cmd.ctrl_mode == 3)//GIM_AUTO
    {
        dynamic_ui_info[Aim].ui_config.color = GREEN;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[Aim]);
    }
    else
    {
        dynamic_ui_info[Aim].ui_config.color = FUCHSIA;
        Enqueue_Ui_For_Sending(&dynamic_ui_info[Aim]);
    }

    uint16_t last_heat = 0;

    if (last_heat != referee.power_heat_data.shooter_17mm_1_barrel_heat)
    {
        static char r[12];
        char *p = r;
        for(int i=0; i<5 && s[i]; i++) *p++ = s[i];
        for(int i=100; i>=1; i/=10) *p++ = '0' + referee.power_heat_data.shooter_17mm_1_barrel_heat/i%10;
        for(int i=0; i<1 && t[i]; i++) *p++ = t[i];
        for(int i=100; i>=1; i/=10) *p++ = '0' + referee.robot_status.shooter_barrel_heat_limit/i%10;
        *p = 0;
        memcpy(dynamic_ui_info[Heat_Indicator].ui_config.text, r, sizeof(r));
        Enqueue_Ui_For_Sending(&dynamic_ui_info[Heat_Indicator]);
        last_heat = referee.power_heat_data.shooter_17mm_1_barrel_heat;
    }
}


