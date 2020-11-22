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
//#include "drv_imu_invensense.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"

/** HITSIC_Module_SYS */
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sys_uartmgr.hpp"
#include "cm_backtrace.h"
//#include "easyflash.h"

/** HITSIC_Module_LIB */
#include "lib_graphic.hpp"

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
#include"ctrl_bal.h"

/*SEND TO UP COMPUTER*/
#include "sc_host.h"
/*PHOTO HEADER FILE*/
#include "image.h"

void MODE_Switch(void);
void MENU_DataSetUp(void);

uint8_t midline=73;
extern uint8_t mid_line[CAMERA_H];
//extern float mid_err;

cam_zf9v034_configPacket_t cameraCfg;
dmadvp_config_t dmadvpCfg;
dmadvp_handle_t dmadvpHandle;
void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);

inv::i2cInterface_t imu_i2c(nullptr, IMU_INV_I2cRxBlocking, IMU_INV_I2cTxBlocking);
inv::mpu6050_t imu_6050(imu_i2c);

disp_ssd1306_frameBuffer_t dispBuffer;
//graphic::bufPrint0608_t<disp_ssd1306_frameBuffer_t> bufPrinter(dispBuffer);//用v 2.0.1版本的模组库就可以消除错误


void main(void)
{
    /** 初始化阶段，关闭总中断 */
     HAL_EnterCritical();

    /** BSP（板级支持包）初始化 */
    RTECLK_HsRun_180MHz();
    RTEPIN_Basic();
    RTEPIN_Digital();
    RTEPIN_Analog();
    RTEPIN_LPUART0_DBG();
    RTEPIN_UART0_WLAN();
    RTEPIP_Basic();
    RTEPIP_Device();

    /** 初始化调试组件 */
    DbgConsole_Init(0U, 921600U, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Welcome to HITSIC !\n");
    PRINTF("GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    cm_backtrace_init("HITSIC_MK66F18", "2020-v3.0", "v4.1.1");

    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    /** 初始化ftfx_Flash */
    FLASH_SimpleInit();
    /** 初始化PIT中断管理器 */
    pitMgr_t::init();
    /** 初始化I/O中断管理器 */
    extInt_t::init();
    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    //DISP_SSD1306_spiDmaInit();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    //DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    /** 初始化菜单 */
    MENU_Init();
    MENU_Data_NvmReadRegionConfig();
    MENU_Data_NvmRead(menu_currRegionNum);
    /** 菜单挂起 */
    MENU_Suspend();
    /** 初始化摄像头 */
    //CAM_ZF9V034_UnitTest();
    cam_zf9v034_configPacket_t cameraCfg;
    CAM_ZF9V034_GetDefaultConfig(&cameraCfg);                                   //设置摄像头配置
    CAM_ZF9V034_CfgWrite(&cameraCfg);                                   //写入配置
    dmadvp_config_t dmadvpCfg;
    CAM_ZF9V034_GetReceiverConfig(&dmadvpCfg, &cameraCfg);    //生成对应接收器的配置数据，使用此数据初始化接受器并接收图像数据。
    DMADVP_Init(DMADVP0, &dmadvpCfg);
    dmadvp_handle_t dmadvpHandle;
    DMADVP_TransferCreateHandle(&dmadvpHandle, DMADVP0, CAM_ZF9V034_DmaCallback);//CAM_ZF9V034_DmaCallback
    uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
    //uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];
    //uint8_t *fullBuffer = NULL;     //之前没有注释掉，总花屏
    disp_ssd1306_frameBuffer_t *dispBuffer = new disp_ssd1306_frameBuffer_t;
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer0);
    //DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer1);
    DMADVP_TransferStart(DMADVP0, &dmadvpHandle);
    //TODO: 在这里初始化摄像头
    /** 初始化IMU */
    if (true != imu_6050.Detect())
    {
        PRINTF("IMU Detection Fail\n");
        while(1);
    }
    if (0U != imu_6050.Init())
    {
        PRINTF("IMU Initialization Fail\n");
        while(1);
    }
    if (0U != imu_6050.SelfTest()) ///> 自检时保持静止，否则会直接失败
    {
        PRINTF("IMU Self Test Fail\n");
        //while(1);
    }
    //TODO: 在这里初始化IMU（MPU6050）
    /** 菜单就绪 */
    MENU_Resume();
    //MENU_Suspend();
    /** 控制环初始化 */
    ctrl_filterInit();
    ctrl_init();
    SendData();
    //TODO: 在这里初始化控制环
    /** 初始化结束，开启总中断 */
    //PORT_SetPinInterruptConfig(PORTA,9U,kPORT_InterruptEitherEdge);//boma按键的触发模式
    //extInt_t::insert(PORTA,9U,MODE_Switch);//中断服务函数
    HAL_ExitCritical();
    /** 内置DSP函数测试 */
    float f = arm_sin_f32(0.6f);


    while (true)
    {
        while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));
        dispBuffer->Clear();
       // if(!GPIO_PinRead(GPIOA,9))
        //{
            //MENU_Suspend();
            //DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
        //}

        const uint8_t imageTH = 160;
        for (int i = 0; i < cameraCfg.imageRow; i += 2)
        {
             int16_t imageRow = i >> 1;//除以2 为了加速;
             int16_t dispRow = (imageRow / 8) + 1, dispShift = (imageRow % 8);
             for (int j = 0; j < cameraCfg.imageCol; j += 2)
             {
                   int16_t dispCol = j >> 1;
                   if (IMG[i][j]> imageTH)//IMG[i][j]
                   {
                        dispBuffer->SetPixelColor(dispCol, imageRow, 1);
                   }
              }
        }
        SCHOST_ImgUpload(fullBuffer,120,188);//fullBuffer是二维数组,这里是列指针，直接输IMG不行  &IMG[0][0]

        SCHOST_ImgUpload(&IMG[0][0],120,188);

        if (!GPIO_PinRead(GPIOA,9))
        {
            MENU_Suspend();
            DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
            if(GPIO_PinRead(GPIOA,9))
            {
                DISP_SSD1306_Init();
                MENU_Resume();
                break;
            }
        }
        /*while(!GPIO_PinRead(GPIOA,9))
                {
            i++;  }
        if(i/2==0||i==0)
        {

            MENU_Resume();
        }
        else
        {
            MENU_Suspend();
              DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
        }*/
        //SendData();
        /*if(GPIO_PinRead(GPIOA,9)==0||GPIO_PinRead(GPIOA,10)==0||GPIO_PinRead(GPIOA,11)==0||GPIO_PinRead(GPIOA,12)==0)
        {
              MENU_Resume();
            GPIO_PinWrite(GPIOE,26,1U);
        }
        else if(GPIO_PinRead(GPIOA,9))
        {
            //MENU_Suspend();
            //DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);//dispBuffer
            GPIO_PinWrite(GPIOE,26,0U);
        }*/
        //DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);//dispBuffer
        DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, fullBuffer);
        DMADVP_TransferStart(DMADVP0,&dmadvpHandle);
        //TODO: 在这里添加车模保护代码
    }
}

void MENU_DataSetUp(void)
{
    ctrl_menuBuild();

    //TODO: 在这里添加子菜单和菜单项
}

void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    //TODO: 补完本回调函数，双缓存采图。
    dmadvp_handle_t *dmadvpHandle = (dmadvp_handle_t*)userData;
    status_t result = 0;

    DMADVP_EdmaCallbackService(dmadvpHandle, transferDone);

    result = DMADVP_TransferStart(dmadvpHandle->base, dmadvpHandle);

    //MENU_Resume();
    //PRINTF("new full buffer: 0x%-8.8x = 0x%-8.8x\n", handle->fullBuffer.front(), handle->xferCfg.destAddr);
    /*if(kStatus_Success != result)
    {
        DMADVP_TransferStop(dmadvpHandle->base, dmadvpHandle);
        PRINTF("transfer stop! insufficent buffer\n");
    }*/
    //TODO: 添加图像处理（转向控制也可以写在这里）
    THRE();
    head_clear();
    image_main(midline);
}
   /*void THRE()
{
    uint8_t* map;
    uint8_t* my_map;
    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > threshold)
                (*my_map) = 255;//255;
            else (*my_map) = 0;
            map++;
            my_map++;
        }
    }
}*/
void MODE_Switch(void)
{
    GPIO_PortToggle(GPIOE,1U<<26);
}


/**
 * 『灯千结的碎碎念』 Tips by C.M. :
 * 1. 浮点数计算有时（例如除零时）会产生“nan”，即“非数（Not-a-Number）”。
 *      要检测一个变量是否为“nan”，只需判断这个变量是否和自身相等。如果该
 *      变量与自身不相等（表达式“var == var”的值为假），则可判定该浮点数
 *      的值是nan，需要进行车模保护动作。
 * 2. 由于车模震动等因素，IMU可能会断开连接。一旦发现IMU读取失败，应执行车
 *      模保护动作。另外，IMU在单片机复位的瞬间可能正在进行传输，导致时序
 *      紊乱，初始化失败。因此装有IMU的车模复位时必须全车断电。
 * 3. 正常情况下图像帧率为50FPS，即20ms一帧。若摄像头时序紊乱，会导致控制周
 *      期混乱。因而有必要在每次图像采集完成时测量距离上次图像采集完成的时
 *      间间隔，如果明显偏离20ms，须执行车模保护动作。
 * 4. 直立车需特别注意：有时控制输出会使两个电机向相反方向旋转，这在正常运行
 *      中是十分危险的，可能造成车模进入“原地陀螺旋转”的状态，极易损坏车模或
 *      导致人员受伤。在设置电机占空比时务必做好异常保护。
 */

