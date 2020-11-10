/*
 * ctrl_bal.h
 *
 *  Created on: 2020年11月12日
 *      Author: MECHREVO
 */

#ifndef CTRL_BAL_H_
#define CTRL_BAL_H_


#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "sc_host.h"

#define ctrl_pi  3.1415926f
#define accelOfGrv  9.8f     //重力加速度
#define divideTg  ctrl_pi    // 1/Tg
#define ctrl_angToRad(x)     (x * (ctrl_pi/ 180.0f))
#define ctrl_radToAng(x)     (x * (180.0f / ctrl_pi))
#define update_Time_ms 5U

typedef struct balance_pidData
{
    float kp;
    float ki;
    float kd;
}PID;

extern inv::mpu6050_t imu_6050;

void ctrl_filterInit(void);
void ctrl_filterUpdata(uint32_t _ms);
void ctrl_balanceContral(void);
void ctrl_dirContorl(void);
void ctrl_speedControl(void);
void ctrl_init(void);
void SendAngle(void);
void ctrl_motorCtrl(float motorL,float motorR);

float smoothavgfilter (float windowsize,float cycle,float data);
void ctrl_menuBuild(void);
float PID_CtrlCal(PID *_pid ,float set , float curr);


#endif /* CTRL_BAL_H_ */

