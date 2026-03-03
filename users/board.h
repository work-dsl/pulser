/**
  ******************************************************************************
  * @copyright   : Copyright To Hangzhou Dinova EP Technology Co.,Ltd
  * @file        : xxx.h
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
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "bsp_conf.h"

/* Exported define -----------------------------------------------------------*/

/* Exported typedef ----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variable prototypes ----------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/
void board_init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __BOARD_H__ */

