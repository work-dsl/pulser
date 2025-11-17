/**
  ******************************************************************************
  * @file        : major_logic.h
  * @brief       : 应用主逻辑协调器接口
  * @details     本文件定义了应用主逻辑协调器的接口。
  *              职责：
  *              - 协调各模块的初始化
  *              - 处理模块间的事件协调
  *              - 处理系统级的状态管理
  * @attention   使用前需要先调用 major_logic_init() 进行初始化
  ******************************************************************************
  */

#ifndef MAJOR_LOGIC_H
#define MAJOR_LOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void major_logic_init(void);
void major_logic_task(void);
void major_logic_request_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* MAJOR_LOGIC_H */

