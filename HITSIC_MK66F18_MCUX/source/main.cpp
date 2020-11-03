/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright 2018 - 2020 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hitsic_common.h"

/** HITSIC_Module_DRV */
#include "drv_ftfx_flash.hpp"
#include "drv_disp_ssd1306.hpp"
#include "drv_imu_invensense.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"

/** HITSIC_Module_SYS */
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sys_uartmgr.hpp"
#include "cm_backtrace.h"
//#include "easyflash.h"

/** HITSIC_Module_APP */
#include "app_menu.hpp"
#include "app_svbmp.hpp"

/** FATFS */
#include "ff.h"
#include "sdmmc_config.h"
FATFS fatfs;                                   //逻辑驱动器的工作区

#include "sc_adc.h"
#include "sc_ftm.h"

/** HITSIC_Module_TEST */
#include "drv_cam_zf9v034_test.hpp"
#include "app_menu_test.hpp"
#include "drv_imu_invensense_test.hpp"
#include "sys_fatfs_test.hpp"
#include "sys_fatfs_diskioTest.hpp"

/** SCLIB_TEST */
#include "sc_test.hpp"


void MENU_DataSetUp(void);

cam_zf9v034_configPacket_t cameraCfg;
dmadvp_config_t dmadvpCfg;
dmadvp_handle_t dmadvpHandle;
void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);

inv::i2cInterface_t imu_i2c(nullptr, IMU_INV_I2cRxBlocking, IMU_INV_I2cTxBlocking);
inv::mpu6050_t imu_6050(imu_i2c);

void main(void)
{
    /** 初始化阶段，关闭总中断 */
    HAL_EnterCritical();
    /** 初始化时钟 */
    RTECLK_HsRun_180MHz();
    /** 初始化引脚路由 */
    RTEPIN_Basic();
    RTEPIN_Digital();
    RTEPIN_Analog();
    RTEPIN_LPUART0_DBG();
    RTEPIN_UART0_WLAN();
    /** 初始化外设 */
    RTEPIP_Basic();
    RTEPIP_Device();
    /** 初始化调试串口 */
    DbgConsole_Init(0U, 921600U, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Welcome to HITSIC !\n");
    PRINTF("GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    /** 初始化CMBackTrace */
    cm_backtrace_init("HITSIC_MK66F18", "2020-v3.0", "v4.1.1");
    /** 初始化ftfx_Flash */
    FLASH_SimpleInit();
    /** 初始化EasyFlash */
    //easyflash_init();
    /** 初始化PIT中断管理器 */
    pitMgr_t::init();
    /** 初始化I/O中断管理器 */
    extInt_t::init();
    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    /** 初始化菜单 */
    MENU_Init();
    MENU_Data_NvmReadRegionConfig();
    MENU_Data_NvmRead(menu_currRegionNum);
    /** 菜单挂起 */
    MENU_Suspend();
    /** 初始化摄像头 */
    //TODO: 在这里初始化摄像头
    /** 初始化IMU */
    //TODO: 在这里初始化IMU（MPU6050）
    /** 菜单就绪 */
    MENU_Resume();
    /** 控制环初始化 */
    //TODO: 在这里初始化控制环
    /** 初始化结束，开启总中断 */
    HAL_ExitCritical();

    float f = arm_sin_f32(0.6f);

    while (true)
    {
        //TODO: 在这里添加车模保护代码
    }
}

void MENU_DataSetUp(void)
{
        static int32_t testInt[8]={1,2,3,4,5,6,7,8};
        static float testFloat[5]={32.03,3.14,9.65,11.54,15.36};
        static menu_list_t *testList_1=MENU_ListConstruct("List_1",10,menu_menuRoot);
        static menu_list_t *testList_2=MENU_ListConstruct("List_2",10,menu_menuRoot);
        assert(testList_1);
        assert(testList_2);
        MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "EXAMPLE", 0, 0));
        MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,testList_1, "List_1", 0, 0));
        MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,testList_2, "List_2", 0, 0));
        //List_1
        MENU_ListInsert(testList_1, MENU_ItemConstruct(nullType, NULL, "", 0, 0));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0],"testInt_1",10,menuItem_data_global));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0]+1,"testInt_2",11,menuItem_data_global));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0]+2,"testInt_3",12,menuItem_data_global));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0]+3,"testInt_4",13,menuItem_data_global));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0]+4,"testInt_5",14,menuItem_data_global));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0]+5,"testInt_6",15,menuItem_data_global));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0]+6,"testInt_7",16,menuItem_data_global));
        MENU_ListInsert(testList_1,MENU_ItemConstruct(variType,&testInt[0]+7,"testInt_8",17,menuItem_data_global));
        //List_2
        MENU_ListInsert(testList_2, MENU_ItemConstruct(nullType, NULL, "test", 0, 0));
        MENU_ListInsert(testList_2,MENU_ItemConstruct(varfType,&testFloat[0],"testFloat_1",0,menuItem_data_region));
        MENU_ListInsert(testList_2,MENU_ItemConstruct(varfType,&testFloat[0]+1,"testFloat_2",1,menuItem_data_region));
        MENU_ListInsert(testList_2,MENU_ItemConstruct(varfType,&testFloat[0]+2,"testFloat_3",2,menuItem_data_region));
        MENU_ListInsert(testList_2,MENU_ItemConstruct(varfType,&testFloat[0]+3,"testFloat_4",3,menuItem_data_region));
        MENU_ListInsert(testList_2,MENU_ItemConstruct(varfType,&testFloat[0]+4,"readOnly",0,menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        //TODO: 在这里添加子菜单和菜单项
}

void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    //TODO: 补完本回调函数

    //TODO: 添加图像处理（转向控制也可以写在这里）
}
