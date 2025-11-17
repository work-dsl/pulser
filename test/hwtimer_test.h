/**
  ******************************************************************************
  * @file        : hwtimer_test.h
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 2025-10-20
  * @brief       : 硬件定时器测试接口
  * @attention   : None
  ******************************************************************************
  */
#ifndef __HWTIMER_TEST_H__
#define __HWTIMER_TEST_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/

/* Exported typedef ----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variable prototypes ----------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief 初始化硬件定时器测试
 * @return 0 成功，负数表示错误码
 */
int hwtimer_test_init(void);

/**
 * @brief 测试单次模式定时器
 */
void hwtimer_test_oneshot(void);

/**
 * @brief 测试周期模式定时器
 */
void hwtimer_test_periodic(void);

/**
 * @brief 测试定时器频率设置
 */
void hwtimer_test_frequency(void);

/**
 * @brief 测试定时器控制接口
 */
void hwtimer_test_control(void);

/**
 * @brief 测试定时器精度
 */
void hwtimer_test_accuracy(void);

/**
 * @brief 运行所有定时器测试
 */
void hwtimer_test_all(void);

/**
 * @brief 清理测试资源
 */
void hwtimer_test_cleanup(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __HWTIMER_TEST_H__ */


