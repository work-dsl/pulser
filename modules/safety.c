/**
  ******************************************************************************
  * @copyright: Copyright To Hangzhou Dinova EP Technology Co.,Ltd
  * @file     : xx.c
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
#include "safety.h"
#include "board.h"
#include "gpio.h"
#include "stimer.h"
#include "watchdog.h"

/*------------------------------ Macro definition -----------------------------*/


/*------------------------------ typedef definition ---------------------------*/

/**
 * @brief 过流保护事件回调函数类型
 */
typedef void (*ocp_event_cb_t)(void);

/*------------------------------ variables prototypes -------------------------*/
stimer_t led_timer;

/**
 * @brief 过流保护事件回调函数指针
 */
static ocp_event_cb_t g_ocp_callback = NULL;

/*------------------------------ function prototypes --------------------------*/
static void OCP_handle(void *args);

/*------------------------------ application ----------------------------------*/
void led_timer_callback(void* arg)
{
    stimer_t *timer = (stimer_t*)arg;
    
    if (gpio_read(LED_PIN_ID) == 1) {
        gpio_write(LED_PIN_ID, 0);
        stimer_change_period(timer, 200);
    } else {
        gpio_write(LED_PIN_ID, 1);
        stimer_change_period(timer, 800);
    }
}

void safety_init(void)
{
    /* set led gpio mode */
    gpio_set_mode(LED_PIN_ID, PIN_OUTPUT_PP, PIN_PULL_UP);
    gpio_write(LED_PIN_ID, 1);
    
    /* 过流保护引脚配置 */
    gpio_set_mode(OCP_POSITIVE_PIN_ID, PIN_INPUT, PIN_PULL_NONE);
    gpio_set_mode(OCP_NEGTIVE_PIN_ID, PIN_INPUT, PIN_PULL_NONE);
    
    gpio_attach_irq (OCP_POSITIVE_PIN_ID, PIN_EVENT_RISING_EDGE, OCP_handle, NULL);
    gpio_irq_enable (OCP_POSITIVE_PIN_ID, 1);
    
    gpio_attach_irq (OCP_NEGTIVE_PIN_ID, PIN_EVENT_RISING_EDGE, OCP_handle, NULL);
    gpio_irq_enable (OCP_NEGTIVE_PIN_ID, 1);
    
    gpio_set_mode(OCP_RESET_PIN_ID, PIN_OUTPUT_PP, PIN_PULL_DOWN);
    gpio_write(OCP_RESET_PIN_ID, 0);
    
    stimer_create(&led_timer, 800, STIMER_AUTO_RELOAD, led_timer_callback, (void*)&led_timer);
    stimer_start(&led_timer);
    
//    MX_IWDG_Init();
}

void safety_task(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

void safety_perform_software_reset(void)
{
    // 触发软件复位
    NVIC_SystemReset();
    
    // 注意：
    // 1. 复位会立即发生。
    // 2. 任何在此函数调用之后的代码都不会被执行。
    // 3. 为防止某些编译器优化或意外情况，可以加一个死循环。
    while (1)
    {
        // 此处代码不应被执行
    }
}

static void OCP_handle(void *args)
{
    (void)args;
    
    /* 调用过流保护回调函数 */
    if (g_ocp_callback != NULL) {
        g_ocp_callback();
    }
    
    /* 上传信息给上位机 */
}

/**
 * @brief 设置过流保护事件回调函数
 * @param callback 回调函数指针
 */
void safety_set_ocp_callback(ocp_event_cb_t callback)
{
    g_ocp_callback = callback;
}

/******************************* End Of File ************************************/
