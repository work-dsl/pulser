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
#include "ocd.h"

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
    /* 底层驱动初始化 */
    board_init();

    /* 系统服务初始化 */
    stimer_init(HAL_GetTick);

    /* 安全模块初始化 */
    safety_init();

    /* 应用主逻辑协调器初始化 */
    major_logic_init();

    /* 协议应用层初始化 */
    slave_proto_init();

    LOG_I("System initialized successfully!");

    while(1)
    {
        safety_task();          /* 安全任务 */
        stimer_service();       /* 软件定时器服务 */
        slave_proto_task();     /* 协议处理任务 */
        major_logic_task();     /* 主逻辑协调任务 */
    }
}

/******************************* End Of File ************************************/


