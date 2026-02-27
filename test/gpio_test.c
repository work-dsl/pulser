/**
 ******************************************************************************
 * @file : gpio_test.c
 * @author : ZJY
 * @version : V1.0
 * @date : 2026-01-24
 * @brief : GPIO单元测试实现
 * @attention : None
 ******************************************************************************
 * @history :
 * V1.0 : 1.初始版本，实现GPIO基本功能和中断功能测试
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gpio_test.h"
#include "gpio.h"
#include "bsp_gpio.h"
#include "errno-base.h"
#include "bsp_conf.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

#define  LOG_TAG             "gpio_test"
#define  LOG_LVL             3
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* 测试使用的GPIO引脚 */
#define TEST_PIN1_ID            25U     /* PB.9 */
#define TEST_PIN2_ID            0U      /* PA.0 */
#define TEST_PIN3_ID            1U      /* PA.1 */
#define TEST_PIN_ID_INVALID     0xFFFFU /* 无效引脚ID */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* 中断回调计数 */
static volatile uint32_t irq_callback_count = 0U;
/* 中断回调参数 */
static volatile void *irq_callback_args = NULL;

/* Exported variables -------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void gpio_irq_test_handler(void *args);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief GPIO中断测试回调函数
 * @param args 回调参数
 * @return None
 */
static void gpio_irq_test_handler(void *args)
{
    irq_callback_count++;
    irq_callback_args = args;
}

/**
 * @brief 初始化GPIO测试
 * @return 0 成功，负数表示错误码
 */
int gpio_test_init(void)
{
    int ret;
    
    LOG_I("========== GPIO Test Initialization ==========");
    
    /* 初始化BSP GPIO */
    bsp_gpio_init();
    
    LOG_I("BSP GPIO initialized");
    LOG_I("GPIO test ready");
    
    return 0;
}

/**
 * @brief 测试GPIO模式设置
 * @return None
 */
void gpio_test_set_mode(void)
{
    LOG_I("========== GPIO Set Mode Test ==========");
    
    /* 测试输入模式 */
    gpio_set_mode(TEST_PIN1_ID, PIN_INPUT, PIN_PULL_NONE);
    LOG_I("Set PA.0 to INPUT mode with no pull");
    
    gpio_set_mode(TEST_PIN1_ID, PIN_INPUT, PIN_PULL_UP);
    LOG_I("Set PA.0 to INPUT mode with pull-up");
    
    gpio_set_mode(TEST_PIN1_ID, PIN_INPUT, PIN_PULL_DOWN);
    LOG_I("Set PA.0 to INPUT mode with pull-down");
    
    /* 测试推挽输出模式 */
    gpio_set_mode(TEST_PIN1_ID, PIN_OUTPUT_PP, PIN_PULL_NONE);
    LOG_I("Set PA.0 to OUTPUT_PP mode");
    
    /* 测试开漏输出模式 */
    gpio_set_mode(TEST_PIN1_ID, PIN_OUTPUT_OD, PIN_PULL_UP);
    LOG_I("Set PA.0 to OUTPUT_OD mode with pull-up");
    
    LOG_I("GPIO set mode test completed");
}

/**
 * @brief 测试GPIO读写功能
 * @return None
 */
void gpio_test_read_write(void)
{
    uint8_t read_value;
    
    LOG_I("========== GPIO Read/Write Test ==========");
    
    /* 配置为推挽输出 */
    gpio_set_mode(TEST_PIN1_ID, PIN_OUTPUT_PP, PIN_PULL_NONE);
    
    /* 测试写高电平 */
    gpio_write(TEST_PIN1_ID, 1U);
    LOG_I("Write PA.0 to HIGH");
    
    /* 延时后读取（如果硬件支持回读） */
    HAL_Delay(10U);
    
    /* 测试写低电平 */
    gpio_write(TEST_PIN1_ID, 0U);
    LOG_I("Write PA.0 to LOW");
    
    HAL_Delay(10U);
    
    /* 配置为输入模式并读取 */
    gpio_set_mode(TEST_PIN2_ID, PIN_INPUT, PIN_PULL_UP);
    read_value = gpio_read(TEST_PIN2_ID);
    LOG_I("Read PA.1 value: %u", read_value);
    
    /* 切换下拉并读取 */
    gpio_set_mode(TEST_PIN2_ID, PIN_INPUT, PIN_PULL_DOWN);
    HAL_Delay(10U);
    read_value = gpio_read(TEST_PIN2_ID);
    LOG_I("Read PA.1 value (with pull-down): %u", read_value);
    
    LOG_I("GPIO read/write test completed");
}

/**
 * @brief 测试GPIO名称解析
 * @return None
 */
void gpio_test_get_pin(void)
{
    int pin_id;
    
    LOG_I("========== GPIO Get Pin Test ==========");
    
    /* 测试有效引脚名称 */
    pin_id = gpio_get("PA.0");
    if (pin_id >= 0) {
        LOG_I("Get pin 'PA.0': %d (expected: 0)", pin_id);
    } else {
        LOG_E("Get pin 'PA.0' failed: %d", pin_id);
    }
    
    pin_id = gpio_get("PB.5");
    if (pin_id >= 0) {
        LOG_I("Get pin 'PB.5': %d (expected: 21)", pin_id);
    } else {
        LOG_E("Get pin 'PB.5' failed: %d", pin_id);
    }
    
    pin_id = gpio_get("PC.15");
    if (pin_id >= 0) {
        LOG_I("Get pin 'PC.15': %d (expected: 47)", pin_id);
    } else {
        LOG_E("Get pin 'PC.15' failed: %d", pin_id);
    }
    
    /* 测试无效引脚名称 */
    pin_id = gpio_get(NULL);
    if (pin_id == -EINVAL) {
        LOG_I("Get pin NULL correctly returned -EINVAL");
    } else {
        LOG_E("Get pin NULL failed: expected -EINVAL, got %d", pin_id);
    }
    
    pin_id = gpio_get("INVALID");
    if (pin_id < 0) {
        LOG_I("Get pin 'INVALID' correctly returned error: %d", pin_id);
    } else {
        LOG_E("Get pin 'INVALID' should fail, got: %d", pin_id);
    }
    
    pin_id = gpio_get("PA");
    if (pin_id < 0) {
        LOG_I("Get pin 'PA' correctly returned error: %d", pin_id);
    } else {
        LOG_E("Get pin 'PA' should fail, got: %d", pin_id);
    }
    
    pin_id = gpio_get("PA.");
    if (pin_id < 0) {
        LOG_I("Get pin 'PA.' correctly returned error: %d", pin_id);
    } else {
        LOG_E("Get pin 'PA.' should fail, got: %d", pin_id);
    }
    
    LOG_I("GPIO get pin test completed");
}

/**
 * @brief 测试GPIO中断附加和分离
 * @return None
 */
void gpio_test_irq_attach_detach(void)
{
    int ret;
    static uint32_t test_args = 0x12345678U;
    
    LOG_I("========== GPIO IRQ Attach/Detach Test ==========");
    
    /* 复位计数器 */
    irq_callback_count = 0U;
    irq_callback_args = NULL;
    
    /* 配置为输入模式 */
    gpio_set_mode(TEST_PIN1_ID, PIN_INPUT, PIN_PULL_UP);
    
    /* 测试附加上升沿中断 */
    ret = gpio_attach_irq(TEST_PIN1_ID, PIN_EVENT_RISING_EDGE, 
                          gpio_irq_test_handler, &test_args);
    if (ret == 0) {
        LOG_I("Attach rising edge IRQ to PA.0: success");
    } else {
        LOG_E("Attach rising edge IRQ to PA.0 failed: %d", ret);
    }
    
    /* 测试重复附加相同配置（应该成功） */
    ret = gpio_attach_irq(TEST_PIN1_ID, PIN_EVENT_RISING_EDGE, 
                          gpio_irq_test_handler, &test_args);
    if (ret == 0) {
        LOG_I("Re-attach same IRQ config: success");
    } else {
        LOG_E("Re-attach same IRQ config failed: %d", ret);
    }
    
    /* 测试分离中断 */
    ret = gpio_detach_irq(TEST_PIN1_ID);
    if (ret == 0) {
        LOG_I("Detach IRQ from PA.0: success");
    } else {
        LOG_E("Detach IRQ from PA.0 failed: %d", ret);
    }
    
    /* 测试再次分离（应该成功，因为已经分离） */
    ret = gpio_detach_irq(TEST_PIN1_ID);
    if (ret == 0) {
        LOG_I("Detach IRQ again: success");
    } else {
        LOG_E("Detach IRQ again failed: %d", ret);
    }
    
    /* 测试附加下降沿中断 */
    ret = gpio_attach_irq(TEST_PIN1_ID, PIN_EVENT_FALLING_EDGE, 
                          gpio_irq_test_handler, &test_args);
    if (ret == 0) {
        LOG_I("Attach falling edge IRQ to PA.0: success");
    } else {
        LOG_E("Attach falling edge IRQ to PA.0 failed: %d", ret);
    }
    
    /* 测试附加双边沿中断 */
    ret = gpio_attach_irq(TEST_PIN2_ID, PIN_EVENT_EITHER_EDGE, 
                          gpio_irq_test_handler, &test_args);
    if (ret == 0) {
        LOG_I("Attach either edge IRQ to PA.1: success");
    } else {
        LOG_E("Attach either edge IRQ to PA.1 failed: %d", ret);
    }
    
    /* 清理 */
    gpio_detach_irq(TEST_PIN1_ID);
    gpio_detach_irq(TEST_PIN2_ID);
    
    LOG_I("GPIO IRQ attach/detach test completed");
}

/**
 * @brief 测试GPIO中断使能控制
 * @return None
 */
void gpio_test_irq_enable(void)
{
    int ret;
    static uint32_t test_args = 0x87654321U;
    
    LOG_I("========== GPIO IRQ Enable Test ==========");
    
    /* 复位计数器 */
    irq_callback_count = 0U;
    irq_callback_args = NULL;
    
    /* 配置为输入模式 */
    gpio_set_mode(TEST_PIN1_ID, PIN_INPUT, PIN_PULL_UP);
    
    /* 先附加中断 */
    ret = gpio_attach_irq(TEST_PIN1_ID, PIN_EVENT_RISING_EDGE, 
                          gpio_irq_test_handler, &test_args);
    if (ret != 0) {
        LOG_E("Attach IRQ failed: %d", ret);
        return;
    }
    
    /* 测试使能中断 */
    ret = gpio_irq_enable(TEST_PIN1_ID, 1U);
    if (ret == 0) {
        LOG_I("Enable IRQ for PA.0: success");
    } else {
        LOG_E("Enable IRQ for PA.0 failed: %d", ret);
    }
    
    /* 等待一段时间，观察中断是否触发 */
    HAL_Delay(100U);
    LOG_I("IRQ callback count after enable: %u", irq_callback_count);
    
    /* 测试禁用中断 */
    ret = gpio_irq_enable(TEST_PIN1_ID, 0U);
    if (ret == 0) {
        LOG_I("Disable IRQ for PA.0: success");
    } else {
        LOG_E("Disable IRQ for PA.0 failed: %d", ret);
    }
    
    /* 等待一段时间，中断不应再触发 */
    uint32_t count_before = irq_callback_count;
    HAL_Delay(100U);
    if (irq_callback_count == count_before) {
        LOG_I("IRQ correctly disabled (no new callbacks)");
    } else {
        LOG_E("IRQ still triggering after disable");
    }
    
    /* 测试未附加中断就使能（应该失败） */
    gpio_detach_irq(TEST_PIN1_ID);
    ret = gpio_irq_enable(TEST_PIN1_ID, 1U);
    if (ret == -ENOSYS) {
        LOG_I("Enable IRQ without attach correctly returned -ENOSYS");
    } else {
        LOG_E("Enable IRQ without attach should fail, got: %d", ret);
    }
    
    /* 清理 */
    gpio_detach_irq(TEST_PIN1_ID);
    
    LOG_I("GPIO IRQ enable test completed");
}

/**
 * @brief 测试GPIO边界条件
 * @return None
 */
void gpio_test_boundary(void)
{
    uint8_t read_value;
    int pin_id;
    
    LOG_I("========== GPIO Boundary Test ==========");
    
    /* 测试边界引脚ID */
    pin_id = gpio_get("PA.0");
    if (pin_id >= 0) {
        gpio_set_mode((uint32_t)pin_id, PIN_INPUT, PIN_PULL_NONE);
        read_value = gpio_read((uint32_t)pin_id);
        LOG_I("Test boundary pin PA.0 (ID: %d), read: %u", pin_id, read_value);
    }
    
    pin_id = gpio_get("PA.15");
    if (pin_id >= 0) {
        gpio_set_mode((uint32_t)pin_id, PIN_INPUT, PIN_PULL_NONE);
        read_value = gpio_read((uint32_t)pin_id);
        LOG_I("Test boundary pin PA.15 (ID: %d), read: %u", pin_id, read_value);
    }
    
    /* 测试不同端口的引脚 */
    pin_id = gpio_get("PB.0");
    if (pin_id >= 0) {
        gpio_set_mode((uint32_t)pin_id, PIN_OUTPUT_PP, PIN_PULL_NONE);
        gpio_write((uint32_t)pin_id, 1U);
        LOG_I("Test port B pin PB.0 (ID: %d)", pin_id);
    }
    
    pin_id = gpio_get("PC.0");
    if (pin_id >= 0) {
        gpio_set_mode((uint32_t)pin_id, PIN_OUTPUT_PP, PIN_PULL_NONE);
        gpio_write((uint32_t)pin_id, 0U);
        LOG_I("Test port C pin PC.0 (ID: %d)", pin_id);
    }
    
    LOG_I("GPIO boundary test completed");
}

/**
 * @brief 测试GPIO错误处理
 * @return None
 */
void gpio_test_error_handling(void)
{
    int ret;
    
    LOG_I("========== GPIO Error Handling Test ==========");
    
    /* 测试无效引脚ID的gpio_get */
    ret = gpio_get(NULL);
    if (ret == -EINVAL) {
        LOG_I("gpio_get(NULL) correctly returned -EINVAL");
    } else {
        LOG_E("gpio_get(NULL) should return -EINVAL, got: %d", ret);
    }
    
    /* 测试无效引脚名称 */
    ret = gpio_get("XX.0");
    if (ret < 0) {
        LOG_I("gpio_get('XX.0') correctly returned error: %d", ret);
    } else {
        LOG_E("gpio_get('XX.0') should fail, got: %d", ret);
    }
    
    /* 测试无效格式 */
    ret = gpio_get("PA0");
    if (ret < 0) {
        LOG_I("gpio_get('PA0') correctly returned error: %d", ret);
    } else {
        LOG_E("gpio_get('PA0') should fail, got: %d", ret);
    }
    
    /* 测试中断操作在不支持的情况下 */
    /* 注意：这取决于BSP实现，如果支持则不会返回-ENOSYS */
    ret = gpio_attach_irq(TEST_PIN1_ID, PIN_EVENT_RISING_EDGE, 
                          gpio_irq_test_handler, NULL);
    if (ret == -ENOSYS) {
        LOG_I("attach_irq correctly returned -ENOSYS (not supported)");
    } else if (ret == 0) {
        LOG_I("attach_irq supported, detaching...");
        gpio_detach_irq(TEST_PIN1_ID);
    } else {
        LOG_E("attach_irq unexpected return: %d", ret);
    }
    
    LOG_I("GPIO error handling test completed");
}

/**
 * @brief 运行所有GPIO测试
 * @return None
 */
void gpio_test_all(void)
{
    int ret;
    
    /* 初始化 */
    ret = gpio_test_init();
    if (ret != 0) {
        LOG_E("GPIO test init failed");
        return;
    }
    
    HAL_Delay(100U);
    
    /* 测试模式设置 */
    gpio_test_set_mode();
    HAL_Delay(100U);
    
    /* 测试读写功能 */
    gpio_test_read_write();
    HAL_Delay(100U);
    
    /* 测试引脚名称解析 */
    gpio_test_get_pin();
    HAL_Delay(100U);
    
    /* 测试边界条件 */
    gpio_test_boundary();
    HAL_Delay(100U);
    
    /* 测试错误处理 */
    gpio_test_error_handling();
    HAL_Delay(100U);
    
    /* 测试中断附加和分离 */
    gpio_test_irq_attach_detach();
    HAL_Delay(200U);
    
    /* 测试中断使能控制 */
    gpio_test_irq_enable();
    HAL_Delay(100U);
    
    LOG_I("========== All GPIO Tests Completed ==========");
}

/**
 * @brief 清理测试资源
 * @return None
 */
void gpio_test_cleanup(void)
{
    /* 分离所有可能的中断 */
    gpio_detach_irq(TEST_PIN1_ID);
    gpio_detach_irq(TEST_PIN2_ID);
    gpio_detach_irq(TEST_PIN3_ID);
    
    /* 复位计数器 */
    irq_callback_count = 0U;
    irq_callback_args = NULL;
    
    LOG_I("GPIO test cleanup completed");
}

/* Private functions ---------------------------------------------------------*/
