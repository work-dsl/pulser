/**
  ******************************************************************************
  * @file        : hwtimer_test.c
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 2025-10-20
  * @brief       : 硬件定时器测试示例
  * @attention   : None
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "hwtimer.h"
#include "bsp_hwtimer.h"
#include "device.h"
#include <stdio.h>

#define  LOG_TAG             "hwtimer_test"
#define  LOG_LVL             4
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static hwtimer_device_t timer2_device;
static bsp_hwtimer_data_t timer2_hw_data;

static volatile uint32_t timer_tick_count = 0;

/* Private function prototypes -----------------------------------------------*/
static void timer_callback(device_t *dev, void *user_data);

/* Test functions ------------------------------------------------------------*/

/**
  * @brief  定时器超时回调函数
  * @param  dev: 设备指针
  * @param  user_data: 用户数据
  * @retval None
  */
static void timer_callback(device_t *dev, void *user_data)
{
    timer_tick_count++;
    
    if (timer_tick_count % 1000 == 0) {
        LOG_I("Timer tick: %u", timer_tick_count);
    }
}

/**
  * @brief  初始化硬件定时器测试
  * @retval 0 成功，负数表示错误码
  */
int hwtimer_test_init(void)
{
    int ret;
    hwtimer_info_t timer_info;
    
    LOG_I("========== Hardware Timer Test ==========");
    
    /* 配置定时器信息 */
    timer_info.maxfreq = 1000000;  /* 1 MHz */
    timer_info.minfreq = 1000;     /* 1 kHz */
    timer_info.maxcnt = 0xFFFF;
    timer_info.cntmode = 0;        /* 递增计数 */
    
    /* 初始化STM32 TIM2硬件 */
    ret = bsp_hwtimer_init(TIM2,
                          1000,             /* 1000 ns (1μs) 分辨率 */
                          TIM2_IRQn,
                          5,                /* 中断优先级 */
                          &timer2_device,
                          &timer2_hw_data);
    
    if (ret != 0) {
        LOG_E("Failed to init TIM2 hardware: %d", ret);
        return ret;
    }
    
    /* 注册定时器设备 */
    ret = hwtimer_register(&timer2_device,
                          "timer2",
                          &stm32_hwtimer_ops,
                          &timer_info,
                          &timer2_hw_data);
    
    if (ret != 0) {
        LOG_E("Failed to register timer2: %d", ret);
        return ret;
    }
    
    /* 打开设备 */
    ret = device_open(&timer2_device.parent, DEVICE_OFLAG_RDWR);
    if (ret != 0) {
        LOG_E("Failed to open timer2: %d", ret);
        return ret;
    }
    
    LOG_I("Timer2 device initialized successfully");
    
    return 0;
}

/**
  * @brief  测试单次模式定时器
  * @retval None
  */
void hwtimer_test_oneshot(void)
{
    int ret;
    
    LOG_I("========== Oneshot Timer Test ==========");
    
    /* 设置回调函数 */
    hwtimer_set_callback(&timer2_device, timer_callback, NULL);
    
    /* 启动单次定时器: 1秒 */
    ret = hwtimer_start(&timer2_device, 1000000, HWTIMER_MODE_ONESHOT);
    if (ret != 0) {
        LOG_E("Failed to start oneshot timer: %d", ret);
        return;
    }
    
    LOG_I("Oneshot timer started (1 second)");
    LOG_I("Waiting for timeout...");
}

/**
  * @brief  测试周期模式定时器
  * @retval None
  */
void hwtimer_test_periodic(void)
{
    int ret;
    
    LOG_I("========== Periodic Timer Test ==========");
    
    /* 设置回调函数 */
    hwtimer_set_callback(&timer2_device, timer_callback, NULL);
    
    /* 启动周期定时器: 1ms */
    ret = hwtimer_start(&timer2_device, 1000, HWTIMER_MODE_PERIOD);
    if (ret != 0) {
        LOG_E("Failed to start periodic timer: %d", ret);
        return;
    }
    
    LOG_I("Periodic timer started (1 ms period)");
    LOG_I("Timer will tick every 1ms...");
}

/**
  * @brief  测试定时器频率设置
  * @retval None
  */
void hwtimer_test_frequency(void)
{
    uint32_t freq;
    uint32_t actual_freq;
    
    LOG_I("========== Timer Frequency Test ==========");
    
    /* 获取当前频率 */
    freq = hwtimer_get_freq(&timer2_device);
    LOG_I("Current frequency: %u Hz", freq);
    
    /* 设置频率为 100 kHz */
    actual_freq = hwtimer_set_freq(&timer2_device, 100000);
    LOG_I("Set frequency to 100 kHz, actual: %u Hz", actual_freq);
    
    /* 设置频率为 10 MHz (超出范围) */
    actual_freq = hwtimer_set_freq(&timer2_device, 10000000);
    LOG_I("Set frequency to 10 MHz, actual: %u Hz (clamped)", actual_freq);
    
    /* 恢复频率为 1 MHz */
    actual_freq = hwtimer_set_freq(&timer2_device, 1000000);
    LOG_I("Restored frequency to 1 MHz, actual: %u Hz", actual_freq);
}

/**
  * @brief  测试定时器控制接口
  * @retval None
  */
void hwtimer_test_control(void)
{
    int ret;
    hwtimer_info_t info;
    uint32_t freq;
    uint32_t count;
    hwtimer_mode_t mode_cfg;
    
    LOG_I("========== Timer Control Interface Test ==========");
    
    /* 获取定时器信息 */
    ret = device_control(&timer2_device.parent, HWTIMER_CTRL_INFO_GET, &info);
    if (ret == 0) {
        LOG_I("Timer info: maxfreq=%u, minfreq=%u, maxcnt=%u",
              info.maxfreq, info.minfreq, info.maxcnt);
    }
    
    /* 获取频率 */
    ret = device_control(&timer2_device.parent, HWTIMER_CTRL_FREQ_GET, &freq);
    if (ret == 0) {
        LOG_I("Current frequency: %u Hz", freq);
    }
    
    /* 设置频率 */
    freq = 500000;
    ret = device_control(&timer2_device.parent, HWTIMER_CTRL_FREQ_SET, &freq);
    if (ret == 0) {
        LOG_I("Set frequency: %u Hz", freq);
    }
    
    /* 通过控制接口启动定时器 */
    mode_cfg.mode = HWTIMER_MODE_PERIOD;
    mode_cfg.timeout_us = 10000;  /* 10ms */
    
    ret = device_control(&timer2_device.parent, HWTIMER_CTRL_START, &mode_cfg);
    if (ret == 0) {
        LOG_I("Timer started via control interface");
    }
    
    /* 延时一段时间 */
    HAL_Delay(100);
    
    /* 获取计数值 */
    ret = device_control(&timer2_device.parent, HWTIMER_CTRL_COUNT_GET, &count);
    if (ret == 0) {
        LOG_I("Current count: %u", count);
    }
    
    /* 停止定时器 */
    ret = device_control(&timer2_device.parent, HWTIMER_CTRL_STOP, NULL);
    if (ret == 0) {
        LOG_I("Timer stopped");
    }
}

/**
  * @brief  测试定时器精度
  * @retval None
  */
void hwtimer_test_accuracy(void)
{
    int ret;
    uint32_t start_tick, end_tick;
    uint32_t elapsed_ms;
    
    LOG_I("========== Timer Accuracy Test ==========");
    
    /* 复位计数器 */
    timer_tick_count = 0;
    
    /* 设置回调 */
    hwtimer_set_callback(&timer2_device, timer_callback, NULL);
    
    /* 启动1ms周期定时器 */
    ret = hwtimer_start(&timer2_device, 1000, HWTIMER_MODE_PERIOD);
    if (ret != 0) {
        LOG_E("Failed to start timer: %d", ret);
        return;
    }
    
    LOG_I("Timer started, measuring for 5 seconds...");
    
    start_tick = HAL_GetTick();
    
    /* 等待5秒 */
    HAL_Delay(5000);
    
    end_tick = HAL_GetTick();
    elapsed_ms = end_tick - start_tick;
    
    /* 停止定时器 */
    hwtimer_stop(&timer2_device);
    
    LOG_I("Expected ticks: ~5000");
    LOG_I("Actual ticks: %u", timer_tick_count);
    LOG_I("Elapsed time: %u ms", elapsed_ms);
    LOG_I("Accuracy: %.2f%%", (float)timer_tick_count / elapsed_ms * 100.0f);
}

/**
  * @brief  运行所有定时器测试
  * @retval None
  */
void hwtimer_test_all(void)
{
    int ret;
    
    /* 初始化 */
    ret = hwtimer_test_init();
    if (ret != 0) {
        LOG_E("Timer test init failed");
        return;
    }
    
    /* 延时 */
    HAL_Delay(100);
    
    /* 测试频率设置 */
    hwtimer_test_frequency();
    HAL_Delay(500);
    
    /* 测试单次模式 */
    hwtimer_test_oneshot();
    HAL_Delay(2000);
    
    /* 测试周期模式 */
    hwtimer_test_periodic();
    HAL_Delay(5000);
    hwtimer_stop(&timer2_device);
    
    /* 测试控制接口 */
    hwtimer_test_control();
    HAL_Delay(500);
    
    /* 测试精度 */
    hwtimer_test_accuracy();
    
    LOG_I("========== All Tests Completed ==========");
}

/**
  * @brief  清理测试资源
  * @retval None
  */
void hwtimer_test_cleanup(void)
{
    /* 停止定时器 */
    hwtimer_stop(&timer2_device);
    
    /* 关闭设备 */
    device_close(&timer2_device.parent);
    
    /* 注销设备 */
    hwtimer_unregister(&timer2_device);
    
    LOG_I("Timer test cleanup completed");
}

/**
  * @brief  TIM2中断服务函数
  * @retval None
  */
void TIM2_IRQHandler(void)
{
    bsp_hwtimer_irq_handler(TIM2);
}


