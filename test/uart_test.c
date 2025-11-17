/**
  ******************************************************************************
  * @file        : xxxx.c
  * @author      : ZJY
  * @version     : V1.0
  * @data        : 20xx-xx-xx
  * @brief       : 
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.xxx
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "uart_test.h"
#include "serial.h"
#include <stdio.h>
#include <string.h>

#define  LOG_TAG             "uart_test"
#define  LOG_LVL             4
#include "log.h"
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Exported variables  -------------------------------------------------------*/
serial_t *port1 = NULL;
serial_t *port2 = NULL;
char w1_buff[] = {"Hello port1!\n"};
uint8_t r1_buff[128];
char w2_buff[] = {"Hello port2!\n"};
uint8_t r2_buff[128];

/* Private function prototypes -----------------------------------------------*/


/* Exported functions --------------------------------------------------------*/
void uart_test_init(void)
{
    int32_t ret;
    uint32_t baud;

    port1 = serial_find("uart1");
    if (!port1) {
        LOG_E("serial port1 find failed");
        return;
    }
    
    port2 = serial_find("uart2");
    if (!port2) {
        LOG_E("serial port2 find failed");
        return;
    }
    
    ret = serial_open(port1);
    if (ret != 0) {
        LOG_E("serial port1 open failed: %d\r\n", ret);
        return;
    }
    
    ret = serial_open(port2);
    if (ret != 0) {
        LOG_E("serial port2 open failed: %d\r\n", ret);
        return;
    }
    
    serial_write(port1, w1_buff, strlen(w1_buff));
    serial_write(port2, w2_buff, strlen(w2_buff));
}

void uart_test_task(void)
{
    int ret;
    static int count = 0;
    
    ret = serial_read(port1, r1_buff, 128);
    if (ret > 0)
    {
        serial_write(port1, r1_buff, ret);
    }
    
    ret = serial_read(port2, r2_buff, 128);
    if (ret > 0)
    {
        serial_write(port2, r2_buff, ret);
    }
}

/* Private functions ---------------------------------------------------------*/


