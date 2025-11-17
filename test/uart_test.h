/**
  ******************************************************************************
  * @file        : serial_test.h
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 2025-10-17
  * @brief       : Serial test header file
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.Serial test
  ******************************************************************************
  */
#ifndef __SERIAL_TEST_H__
#define __SERIAL_TEST_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/

/* Exported typedef ----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variable prototypes ----------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/
void uart_test_init(void);
void uart_test_task(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SERIAL_TEST_H__ */

