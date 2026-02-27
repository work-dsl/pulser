/**
 ******************************************************************************
 * @file : gpio_test.h
 * @author : ZJY
 * @version : V1.0
 * @date : 2026-01-24
 * @brief : GPIO单元测试接口
 * @attention : None
 ******************************************************************************
 * @history :
 * V1.0 : 1.初始版本，实现GPIO基本功能和中断功能测试
 *
 *
 ******************************************************************************
 */
#ifndef __GPIO_TEST_H__
#define __GPIO_TEST_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化GPIO测试
 * @return 0 成功，负数表示错误码
 */
int gpio_test_init(void);

/**
 * @brief 测试GPIO模式设置
 */
void gpio_test_set_mode(void);

/**
 * @brief 测试GPIO读写功能
 */
void gpio_test_read_write(void);

/**
 * @brief 测试GPIO名称解析
 */
void gpio_test_get_pin(void);

/**
 * @brief 测试GPIO中断附加和分离
 */
void gpio_test_irq_attach_detach(void);

/**
 * @brief 测试GPIO中断使能控制
 */
void gpio_test_irq_enable(void);

/**
 * @brief 测试GPIO边界条件
 */
void gpio_test_boundary(void);

/**
 * @brief 测试GPIO错误处理
 */
void gpio_test_error_handling(void);

/**
 * @brief 运行所有GPIO测试
 */
void gpio_test_all(void);

/**
 * @brief 清理测试资源
 */
void gpio_test_cleanup(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __GPIO_TEST_H__ */
