/*
 * ctrl_bal.c
 *
 *  Created on: 2020年11月12日
 *      Author: MECHREVO
 */
#include"ctrl_bal.h"


float imu_accel[3]={0};      //从陀螺仪读取的加速度值
float imu_palst[3]={0};      //从陀螺仪读取的角速度值
float filterAngle=0.0f;      //滤波后的角度
float accelAngle=0.0f;       //由加速度计算出的角度值
float palstAngle=0.0f;       //由角速度直接积分得出的角度
float ininAngle=0.0f;        //滤波初始化时的角度
float AngleSet=21.35f;       //不变的机械零点
float angleSet=0.0f;         //机械零点角度
float angelOutput = 0.0f;    //pid计算后的输出值
float limitPWMRear=50.0f;    //电机反转限幅
float limitPWMFront=-50.0f;  //电机正转限幅
float limitTurPwm=5.0f;     ////转向环输出限幅

float mid_err = 0.0f;
float divide_rSet = 0.0f;
float rSet=0.0f;
float w_set = 0.0f;
float w_filter = 0.0f;
float speed = 0.0f;
float pwm_diff = 0.0f;
float kp_midErr = 0.0f;


float smooth_speed=0.0f;
float speed_set=0.0f;
float get_speedl=0.0f;
float get_speedr=0.0f;
float get_speed=0.0f;
float speed_PIDOUTPUT=0.0f;
extern uint8_t mid_line[CAMERA_H];

PID balaPid = {0.0f,0.0f,0.0f};
PID dirPid = {0.0f,0.0f,0.0f};
PID speedPid = {0.0f,0.0f,0.0f};

/**
 * @brief : 滤波初始化函数
 *用于给滤波一个最初角度值
 *
 * @param void
 */
void ctrl_filterInit(void)
{
    const uint32_t sampleTime = 1000;
    float sumAngle = 0.0f;
    for(uint32_t i = 0; i < sampleTime; ++i)
    {
         imu_6050.ReadSensorBlocking();
         imu_6050.Convert(&imu_accel[0], &imu_accel[1], &imu_accel[2], &imu_palst[0], &imu_palst[1], &imu_palst[2]);
         float accelAx=imu_accel[2];
         accelAx = (accelAx> accelOfGrv - 0.001) ? (accelOfGrv - 0.001) : accelAx;
         accelAx = (accelAx < - (accelOfGrv - 0.001)) ? (- (accelOfGrv - 0.001)) : accelAx;
         accelAngle = ctrl_radToAng(- asin(accelAx/ accelOfGrv));
         sumAngle += accelAngle;
         SDK_DelayAtLeastUs(1000,CLOCK_GetFreq(kCLOCK_CoreSysClk)); // MPU6050更新速度没那么快，连续读取只会得到相同的值。
     }
     ininAngle = sumAngle / ((float)sampleTime);
     filterAngle = ininAngle;
     palstAngle = ininAngle;
}

/**
 * @brief : 滤波角度更新函数
 *用于滤波
 *
 * @param  uint32_t _ms  循环周期
 */
void ctrl_filterUpdata(uint32_t _ms)
{
    float dt=((float)_ms)*0.001f;
    float accelAx=imu_accel[2];//1-测与平衡车y轴夹角
    float palstAx=imu_palst[1];//2-平衡车z轴角速度     1-平衡车y轴角速度
    accelAx = (accelAx> accelOfGrv - 0.001) ? (accelOfGrv - 0.001) : accelAx;
    accelAx = (accelAx < - (accelOfGrv - 0.001)) ? (- (accelOfGrv - 0.001)) : accelAx;
    accelAngle = ctrl_radToAng(- asin(accelAx/ accelOfGrv));
    palstAngle+=palstAx*dt;
    filterAngle+=(palstAx+(accelAngle-filterAngle)*divideTg)*dt;//滤波
}

/**
 * @brief : pid计算函数
 *
 * @param  uint32_t update_Time_ms  循环时间
 */
float PID_CtrlCal(PID *_pid ,float set , float curr)
{
    float dT = ((float)update_Time_ms) * 0.001f;
    float set_diff = set * dT;
    float curr_diff = curr * dT;
    float set_intg = 0 , curr_intg = 0;
    set_intg += set_diff;
    curr_intg += curr_diff;
    return  _pid -> kp * (set - curr) + _pid -> kd * (set_diff - curr_diff) + _pid -> ki * (set_intg - curr_intg);
}

/**
 * @brief : 控制平衡的中断服务函数
 *
 * @param  void
 */
void ctrl_balanceContral(void)
{

    imu_6050.ReadSensorBlocking();
    imu_6050.Convert(&imu_accel[0], &imu_accel[1], &imu_accel[2], &imu_palst[0], &imu_palst[1], &imu_palst[2]);
    ctrl_filterUpdata(5U);
    angelOutput = PID_CtrlCal(&balaPid,angleSet,filterAngle);
    angelOutput = angelOutput > limitPWMRear? limitPWMRear: angelOutput;
    angelOutput = angelOutput < limitPWMFront? limitPWMFront: angelOutput;
    ctrl_motorCtrl(angelOutput + pwm_diff,angelOutput - pwm_diff);
}

/**
 * @brief : 直立环的中断
 *
 * @param  void
 */
void ctrl_init(void)
{
    pitMgr_t::insert(5U,2,ctrl_dirContorl, pitMgr_t::enable);
    pitMgr_t::insert(20U,5,ctrl_speedControl, pitMgr_t::enable);
    pitMgr_t::insert(5U,3,ctrl_balanceContral, pitMgr_t::enable);
}

/**
 * @brief : 控制电机输出函数
 *
 * @param  float motorL  右电机输入
 * @param  float motorR  左电机输入
 */
void ctrl_motorCtrl(float motorL,float motorR)
{
    if(motorL < 0.0f && motorR > 0.0f)
    {
        motorL = 0.0f;
    }
    if(motorL > 0.0f && motorR < 0.0f)
    {
        motorR = 0.0f;
    }
    if(motorL>0.0f)
    {
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0, 20000U, motorL);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U, -motorL);
    }

    if(motorR > 0.0f)
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, motorR);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, -motorR);
    }

}




void ctrl_dirContorl(void)
{
    divide_rSet = mid_err * kp_midErr;
    rSet=1/divide_rSet;
    w_set = get_speed *divide_rSet;
    w_filter = ctrl_angToRad(imu_palst[0]);//这里直接读取的是度/s需要转换成弧度/s
    pwm_diff = PID_CtrlCal(&dirPid,w_set,w_filter);
    pwm_diff = pwm_diff > limitTurPwm? limitTurPwm: pwm_diff;
    pwm_diff = pwm_diff < -limitTurPwm? -limitTurPwm: pwm_diff;
}

void ctrl_speedControl(void)
{
    get_speedl=((float)SCFTM_GetSpeed(ENCO_L_PERIPHERAL))*encoderTrs;
    SCFTM_ClearSpeed(ENCO_L_PERIPHERAL);
    get_speedr=-((float)SCFTM_GetSpeed(ENCO_R_PERIPHERAL))*encoderTrs;
    SCFTM_ClearSpeed(ENCO_R_PERIPHERAL);
    get_speed=(get_speedl+get_speedr)/2.0f;
    speed_PIDOUTPUT=PID_CtrlCal(&speedPid,speed_set,get_speed);
    smoothavgfilter (speed_PIDOUTPUT);
    angleSet=AngleSet+smooth_speed;//修改这里的angleSet解决了一直以来的问题
}
float smoothavgfilter (float data)
{
        int NUM=10;//平滑滤波个数
        //NUM=((int)windowsize/cycle);
        static float buf[10]={0};
        static int index=0,flag=0;
        static float sum=0.0f;
        sum += data - buf[index];
        buf[index] = data;// 替换之前位置上的数据
        index++;
        // 控制数据循环放置到缓冲区
        if(index==NUM)
        {
            index = 0;
            flag = 1;
        }
        // 如果没有充满缓存区，有几个就取几个的平均
        if(flag==0)
        { smooth_speed=sum/index;
        //return smooth_speed;
        }
        else
        { smooth_speed=sum/NUM;
        //return smooth_speed;
        }
        return smooth_speed;
  }

/**
 * @brief : 给上位机发角度的函数
 *
 * @param  void
 */
void SendData(void)
{
    float data[4]={mid_err,w_filter,w_set,1/divide_rSet};
    SCHOST_VarUpload(data,4);
}


/**
 * @brief : 参数菜单函数
 *
 * @param  void
 */
void ctrl_menuBuild(void)
{
            static menu_list_t *ctrlList_1=MENU_ListConstruct("List_1",25,menu_menuRoot);
            assert(ctrlList_1);
            MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "EXAMPLE", 0, 0));
            MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,ctrlList_1, "ctrlList_1", 0, 0));
            //List_1
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&rSet,"rSet",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(nullType,NULL, "", 0, 0));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&mid_err,"miderr",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&balaPid.kp,"balakp",10,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&balaPid.kd,"balakd",11,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&balaPid.ki,"balaki",12,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&angleSet,"angleSet",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&AngleSet,"AngleSet",13,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&filterAngle,"filterAngle",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&w_filter,"W",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&limitPWMRear,"limPWMRear",14,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&limitPWMFront,"limPWMFrot",15,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&limitTurPwm,"limTurPwm",1,menuItem_data_region));//转弯差速限幅
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&dirPid.kp,"dirkp",16,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&dirPid.ki,"dirki",17,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&dirPid.kd,"dirkd",18,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&speedPid.kp,"speedkp",19,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&speedPid.ki,"speedki",20,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&speedPid.kd,"speedkd",21,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&kp_midErr,"midErrkp",22,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&get_speedl,"speedl",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&get_speedr,"speedr",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&speed_set,"speedset",23,menuItem_data_global));


            //TODO: 在这里添加子菜单和菜单项
}
