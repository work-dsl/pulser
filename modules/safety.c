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

/*------------------------------ variables prototypes -------------------------*/
stimer_t led_timer;

static struct watchdog_device* iwdg_dev = NULL;

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


    stimer_create(&led_timer, 800, STIMER_AUTO_RELOAD, led_timer_callback, (void*)&led_timer);
    stimer_start(&led_timer);

    iwdg_dev = watchdog_find("iwdg");
    if (iwdg_dev != NULL) {
        watchdog_set_timeout(iwdg_dev, 1000);
        watchdog_start(iwdg_dev);
    }
}

void safety_task(void)
{
    watchdog_ping(iwdg_dev);
}

void safety_perform_software_reset(void)
{
    __disable_irq();

    // 确保 Flash 操作完成
    FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

    IWDG->KR = 0xAAAA;

    // 触发软件复位
    NVIC_SystemReset();

    // 复位指令发出后，CPU 还需要几个时钟周期才能真正复位
    // 加个死循环防止 CPU 继续往下乱跑
    while(1) {}
}


/******************************* End Of File ************************************/
