/**
  ******************************************************************************
  * @copyright: Copyright To Hangzhou Dinova EP Technology Co.,Ltd
  * @file     : main.c
  * @author   : ZJY
  * @version  : V1.0
  * @date     : 20xx-xx-xx
  * @brief    : xxx
  *                  1.xx
  *                  2.xx
  *
  * @attention: None
  ******************************************************************************
  * @history  : 
  *      V1.0 : 1.xxx
  *
  *
  *     
  ******************************************************************************
  */
/*------------------------------ include --------------------------------------*/
#include "board.h"
#include "custom_slave.h"
#include "major_logic.h"
#include "bsp_adc.h"
#include "stimer.h"
#include "safety.h"
#include "current_monitor.h"

/* 测试头文件 */
#include "led_test.h"
#include "uart_test.h"

#define  LOG_TAG             "main"
#define  LOG_LVL             4
#include "log.h"

/*------------------------------ Macro definition -----------------------------*/

/*------------------------------ typedef definition ---------------------------*/


/*------------------------------ variables prototypes -------------------------*/

/*------------------------------ function prototypes --------------------------*/

/*------------------------------ application ----------------------------------*/
/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{   
    int ret;
    
    /* 底层驱动初始化 */
    board_init();
    
    /* 系统服务初始化 */
    stimer_init(HAL_GetTick);
    
    /* 安全模块初始化 */
    safety_init();
    
    /* 应用主逻辑协调器初始化（初始化所有业务模块） */
    major_logic_init();
    
    /* 协议应用层初始化 */
    slave_proto_init();
    
    /* 初始化电流监控模块（使用内部定义的缓冲区） */
    ret = current_monitor_init();
    if (ret != 0) {
        LOG_E("Failed to init current monitor: %d", ret);
    }
    
    /* 启动ADC DMA循环采样 */
    ret = current_monitor_start();
    if (ret != 0) {
        LOG_E("Failed to start current monitor: %d", ret);
    }
    
    LOG_I("System initialized successfully!");
    
    while(1)
    {
        stimer_service();       /* 软件定时器服务 */
        slave_proto_task();     /* 协议处理任务 */
        major_logic_task();     /* 主逻辑协调任务 */
        safety_task();
    }
}

/******************************* End Of File ************************************/


