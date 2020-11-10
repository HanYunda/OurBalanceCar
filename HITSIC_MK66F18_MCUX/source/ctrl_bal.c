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
float angleSet=21.35f;       //机械零点角度
float angelOutput = 0.0f;    //pid计算后的输出值
float limitPWMRear=90.0f;    //电机反转限幅
float limitPWMFront=-90.0f;  //电机正转限幅


typedef struct balance_pidData
{
    float kp;
    float ki;
    float kd;
}DATA;
DATA balaPid = {0.0f,0.0f,0.0f};

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
void ctrl_pid(uint32_t update_Time_ms)
{
    float dT = ((float)update_Time_ms) * 0.001f;
    float angelSetDiff = angleSet * dT;
    float angelFilterDiff = filterAngle * dT;
    angelOutput = balaPid.kp*(angleSet - filterAngle) + balaPid.kd * (angelSetDiff - angelFilterDiff);
    angelOutput = angelOutput > limitPWMRear? limitPWMRear: angelOutput;
    angelOutput = angelOutput < limitPWMFront? limitPWMFront: angelOutput;
    //ctrl_motorCtrl(angelOutput,angelOutput);
}

/**
 * @brief : 控制平衡的终端服务函数
 *
 * @param  void
 */
void ctrl_balanceContral(void)
{
    imu_6050.ReadSensorBlocking();
    imu_6050.Convert(&imu_accel[0], &imu_accel[1], &imu_accel[2], &imu_palst[0], &imu_palst[1], &imu_palst[2]);
    ctrl_filterUpdata(5U);
    ctrl_pid(5U);
    ctrl_motorCtrl(angelOutput,angelOutput);
}

/**
 * @brief : 直立环的中断
 *
 * @param  void
 */
void ctrl_init(void)
{
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

/**
 * @brief : 给上位机发角度的函数
 *
 * @param  void
 */
void SendAngle(void)
{
    float angle[3]={accelAngle,palstAngle,filterAngle};
    SCHOST_VarUpload(angle,3);
}


/**
 * @brief : 参数菜单函数
 *
 * @param  void
 */
void ctrl_menuBuild(void)
{
            static menu_list_t *ctrlList_1=MENU_ListConstruct("List_1",10,menu_menuRoot);
            assert(ctrlList_1);
            MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "EXAMPLE", 0, 0));
            MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,ctrlList_1, "ctrlList_1", 0, 0));
            //List_1
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(nullType,NULL, "", 0, 0));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&balaPid.kp,"kp",10,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&balaPid.kd,"kd",11,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&balaPid.ki,"ki",12,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&angleSet,"angleSet",13,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&filterAngle,"filterAngle",0,menuItem_data_ROFlag|menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&limitPWMRear,"limPWMRear",14,menuItem_data_global));
            MENU_ListInsert(ctrlList_1,MENU_ItemConstruct(varfType,&limitPWMFront,"limPWMFrot",15,menuItem_data_global));
            //TODO: 在这里添加子菜单和菜单项
}


