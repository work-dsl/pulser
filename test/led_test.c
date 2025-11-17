/**
  ******************************************************************************
  * @file        : led_test.c
  * @author      : ZJY
  * @version     : V1.0
  * @data        : 2025-10-17
  * @brief       : LED test
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.LED test
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "led_test.h"
#include "gpio.h"
#include "bsp_conf.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
uint8_t level1 = 0;
uint8_t level2 = 0;
uint8_t level3 = 0;

/* Private variables ---------------------------------------------------------*/


/* Exported variables  -------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void led_test_init(void)
{
    gpio_set_mode(LED_PIN_ID, PIN_OUTPUT_PP, PIN_PULL_UP);
    gpio_write(25, 1);
    
    gpio_set_mode(20, PIN_INPUT, PIN_PULL_NONE);
    gpio_set_mode(21, PIN_INPUT, PIN_PULL_NONE);
}



void led_test_task(void)
{   
//    gpio_write(25, 0);
//    printf("hello!\r\n");
//    HAL_Delay(500);
//    gpio_write(25, 1);
//    HAL_Delay(500);
    level1 = gpio_read(20);
    level2 = gpio_read(21);
    level3 = gpio_read(22);
}

/* Private functions ---------------------------------------------------------*/

