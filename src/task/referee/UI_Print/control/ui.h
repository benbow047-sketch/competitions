#ifndef __UI_H
#define __UI_H

#include "main.h"
#include "rm_task.h"

typedef enum{
    Frc,
    Rot,
    Aim,
    Heat_Indicator,
    Cross_H,//1
    Cross_V,//2
    Aim_Frame,//3
    Carline_L,//4
    Carline_R,//5
	///测试ui 无意义///
	// rectangle1,//6正常
	// rectangle2,//7正常
	// rectangle3,//8小bug
	// rectangle4,//9同3
	// rectangle5,//10大bug
	// rectangle6,//11 3消失 5出现
	// rectangle7,//12 5仍有
	// rectangle8,//13 5仍有
	// rectangle9,//14 5消失
	// rectangle10,//15 小bug
	// rectangle11,//16 小bug
	// rectangle12,//17 小bug
	// rectangle13,//18 正常
	// rectangle14,//19 正常
	// rectangle15,//20 正常
	DYNAMIC_UI_NUM,
}dynamic_ui_cnt_e;


void My_Ui_Init(void);
void Ui_Info_Update(struct gimbal_cmd_msg gim_cmd,struct chassis_cmd_msg chassis_cmd,struct shoot_cmd_msg shoot_cmd,struct referee_msg referee);
#endif