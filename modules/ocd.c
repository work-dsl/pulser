/**
  ******************************************************************************
  * @file        : ocd.c
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 20xx-xx-xx
  * @brief       : 过流检测模块实现
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.初始版本
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "ocd.h"
#include "bsp_adc.h"
#include "errno-base.h"
#include "bsp_comp.h"
#include "bsp_dac.h"
#include "board.h"
#include "gpio.h"
#include "bsp_dwt.h"

#define  LOG_TAG             "ocd"
#define  LOG_LVL             4
#include "log.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

uint16_t adc1_dma_buf[256] = {0};
uint16_t adc2_dma_buf[256] = {0};

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief 通道信息结构体
 */
typedef struct {
    uint16_t *buffer;               /**< DMA缓冲区指针 */
    uint16_t buf_size;              /**< 缓冲区大小（元素个数） */
    volatile uint16_t trigger_pos;  /**< 触发点在缓冲区中的位置 */
    volatile uint8_t trigger_flag;  /**< 触发标志 */
} channel_info_t;

/* Private define ------------------------------------------------------------*/

#define SEARCH_WINDOW_SIZE    (64U)  /**< 峰值搜索窗口大小（触发点前后64个点） */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 通道信息数组
 */
static channel_info_t g_channel_info[2];

/**
 * @brief 过流事件回调函数（COMP+DAC检测方式）
 */
static ocp_info_cb_t g_ocp_callback = NULL;

/**
 * @brief 初始化标志
 */
static uint8_t g_initialized = 0U;

/**
 * @brief 过流阈值存储（单位：mV）
 */
static uint16_t g_ocd_threshold_ch1 = 0U;  /**< 通道1阈值 */
static uint16_t g_ocd_threshold_ch2 = 0U;  /**< 通道2阈值 */

/* Exported variables  -------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint16_t get_dma_current_position(DMA_HandleTypeDef *hdma, uint16_t *buf, uint16_t buf_size);
static uint16_t find_peak_in_window(uint16_t *buf, uint16_t buf_size, uint16_t center_pos, uint16_t window_size);
static uint16_t adc_value_to_voltage(uint16_t adc_value);
static uint16_t voltage_to_dac_value(uint16_t voltage_mv);
static void ocp_io_handle(void *args);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化过流检测模块
 * @note 使用内部定义的缓冲区，无需外部传入
 */
int ocd_init(void)
{
    /* 缓冲区初始化 */
    g_channel_info[OCD_POSITIVE].buffer = adc1_dma_buf;
    g_channel_info[OCD_POSITIVE].buf_size = (uint16_t)sizeof(adc1_dma_buf) / sizeof(adc1_dma_buf[0]);
    g_channel_info[OCD_POSITIVE].trigger_pos = 0U;
    g_channel_info[OCD_POSITIVE].trigger_flag = 0U;

    g_channel_info[OCD_NEGTIVE].buffer = adc2_dma_buf;
    g_channel_info[OCD_NEGTIVE].buf_size = (uint16_t)sizeof(adc2_dma_buf) / sizeof(adc2_dma_buf[0]);
    g_channel_info[OCD_NEGTIVE].trigger_pos = 0U;
    g_channel_info[OCD_NEGTIVE].trigger_flag = 0U;

    /* 过流保护IO引脚配置 */
    gpio_set_mode(OCP_POSITIVE_PIN_ID, PIN_INPUT, PIN_PULL_NONE);
    gpio_set_mode(OCP_NEGTIVE_PIN_ID, PIN_INPUT, PIN_PULL_NONE);

    gpio_set_mode(OCP_RESET_PIN_ID, PIN_OUTPUT_PP, PIN_PULL_DOWN);
    gpio_write(OCP_RESET_PIN_ID, 0);

    bsp_adc1_init();
    bsp_adc2_init();

    MX_DAC1_Init();
    MX_DAC3_Init();

    MX_COMP1_Init();
    MX_COMP3_Init();

    g_initialized = 1U;

    LOG_D("OCD initialized: CH1 buf_size=%d, CH2 buf_size=%d",
          g_channel_info[OCD_POSITIVE].buf_size,
          g_channel_info[OCD_NEGTIVE].buf_size);

    return 0;
}

/**
 * @brief 启动ADC DMA循环采样
 */
int ocd_start(void)
{
    HAL_StatusTypeDef status;

    if (g_initialized == 0U) {
        return -ENODEV;
    }

    /* 启动ADC DMA循环采样 */
    status = HAL_ADC_Start_DMA(&hadc1,
                                (uint32_t*)g_channel_info[OCD_POSITIVE].buffer,
                                g_channel_info[OCD_POSITIVE].buf_size);
    if (status != HAL_OK) {
        LOG_E("Failed to start ADC1 DMA: %d", status);
        return -EIO;
    }
    status = HAL_ADC_Start_DMA(&hadc2,
                                (uint32_t*)g_channel_info[OCD_NEGTIVE].buffer,
                                g_channel_info[OCD_NEGTIVE].buf_size);
    if (status != HAL_OK) {
        LOG_E("Failed to start ADC2 DMA: %d", status);
        return -EIO;
    }

    /* 启动 DAC */
    status = HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    if (status != HAL_OK) {
        LOG_E("Failed to start DAC1: %d", status);
        return -EIO;
    }
    status = HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
    if (status != HAL_OK) {
        LOG_E("Failed to start DAC3: %d", status);
        return -EIO;
    }
    status = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
    if (status != HAL_OK) {
        LOG_E("Failed to set DAC1 value: %d", status);
        return -EIO;
    }
    bsp_dwt_delay_us(20); /* 等待电压建立 */
    status = HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
    if (status != HAL_OK) {
        LOG_E("Failed to set DAC3 value: %d", status);
        return -EIO;
    }
    bsp_dwt_delay_us(20); /* 等待电压建立 */

    status = HAL_COMP_Start(&hcomp1);
    if (status != HAL_OK)
    {
        LOG_E("Failed to start COMP1: %d", status);
        return -EIO;
    }

    status = HAL_COMP_Start(&hcomp3);
    if (status != HAL_OK)
    {
        LOG_E("Failed to start COMP3: %d", status);
        return -EIO;
    }

    return 0;
}

/**
 * @brief 停止ADC DMA循环采样
 */
void ocd_stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    LOG_D("ADC DMA stopped");
}

/**
 * @brief 设置过流事件回调函数（COMP+DAC检测方式）
 */
void ocd_set_comp_callback(ocp_info_cb_t callback)
{
    g_ocp_callback = callback;
}

/**
 * @brief 设置过流保护事件回调函数（IO引脚检测方式）
 * @param callback 回调函数指针
 */
void ocd_set_io_callback(ocp_io_cb_t callback)
{
    gpio_attach_irq(OCP_POSITIVE_PIN_ID, PIN_EVENT_RISING_EDGE, callback, NULL);
    gpio_irq_enable(OCP_POSITIVE_PIN_ID, 1);

    gpio_attach_irq(OCP_NEGTIVE_PIN_ID, PIN_EVENT_RISING_EDGE, callback, NULL);
    gpio_irq_enable(OCP_NEGTIVE_PIN_ID, 1);
}

/**
 * @brief 处理过流事件（在比较器中断中调用）
 * @details 过流发生后立即停止ADC采集，标记DMA位置
 */
void ocd_handle_ocp_trigger(ocd_ch_t channel)
{
    channel_info_t *ch_info;
    uint16_t dma_pos;

    if (channel >= 2U) {
        return;
    }

    ch_info = &g_channel_info[channel];

    /* 立即停止对应通道的ADC DMA采集，保持缓冲区数据不变 */
    if (channel == OCD_POSITIVE) {
        HAL_ADC_Stop_DMA(&hadc1);
        /* 获取停止时的DMA位置 */
        dma_pos = get_dma_current_position(&hdma_adc1, ch_info->buffer, ch_info->buf_size);
    } else {
        HAL_ADC_Stop_DMA(&hadc2);
        /* 获取停止时的DMA位置 */
        dma_pos = get_dma_current_position(&hdma_adc2, ch_info->buffer, ch_info->buf_size);
    }

    /* 标记触发位置和标志 */
    ch_info->trigger_pos = dma_pos;
    ch_info->trigger_flag = 1U;

    LOG_D("OCP triggered: CH%d, stopped at pos=%d", channel, dma_pos);
}

/**
 * @brief 处理过流事件（在任务中调用）
 * @details 使用停止ADC采集前的缓冲区数据进行峰值查找
 */
void ocd_process_ocp_event(void)
{
    uint8_t ch;
    channel_info_t *ch_info;
    ocp_info_t ocp_info;
    uint16_t peak_adc_value;
    uint16_t peak_pos;
    uint16_t voltage_mv;

    if (g_ocp_callback == NULL) {
        return;
    }

    /* 遍历所有通道，处理触发事件 */
    for (ch = 0U; ch < 2U; ch++) {
        ch_info = &g_channel_info[ch];

        if (ch_info->trigger_flag != 0U) {
            /* 清除标志 */
            ch_info->trigger_flag = 0U;

            /* 在触发点附近查找峰值（使用停止ADC采集前的缓冲区数据） */
            peak_pos = find_peak_in_window(ch_info->buffer,
                                          ch_info->buf_size,
                                          ch_info->trigger_pos,
                                          SEARCH_WINDOW_SIZE);
            peak_adc_value = ch_info->buffer[peak_pos];

            /* 将ADC值转换为电压值（毫伏） */
            voltage_mv = adc_value_to_voltage(peak_adc_value);

            /* 构建过流信息 */
            ocp_info.channel = (ocd_ch_t)ch;
            ocp_info.voltage_mv = voltage_mv;

            /* 调用回调函数上报 */
            if (g_ocp_callback != NULL) {
                g_ocp_callback(&ocp_info);
            }

            LOG_I("OCP processed: CH%d, peak_adc=%d, voltage=%d mV", ch, peak_adc_value, voltage_mv);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 获取DMA当前传输位置
 * @param hdma DMA句柄
 * @param buf 缓冲区指针
 * @param buf_size 缓冲区大小
 * @return 当前位置索引（0 ~ buf_size-1）
 */
static uint16_t get_dma_current_position(DMA_HandleTypeDef *hdma, uint16_t *buf, uint16_t buf_size)
{
    uint32_t remaining;
    uint32_t transferred;
    uint16_t pos;

    /* 读取DMA剩余传输数量 */
    remaining = __HAL_DMA_GET_COUNTER(hdma);

    /* 计算已传输数量 */
    transferred = buf_size - remaining;

    /* 计算当前位置（考虑循环） */
    pos = (uint16_t)(transferred % buf_size);

    return pos;
}

/**
 * @brief 在窗口内查找峰值
 * @param buf 缓冲区指针
 * @param buf_size 缓冲区大小
 * @param center_pos 中心位置（触发点）
 * @param window_size 搜索窗口大小
 * @return 峰值位置索引
 */
static uint16_t find_peak_in_window(uint16_t *buf, uint16_t buf_size, uint16_t center_pos, uint16_t window_size)
{
    uint16_t start_pos, end_pos;
    uint16_t i;
    uint16_t peak_pos = center_pos;
    uint16_t peak_value = buf[center_pos];
    uint16_t half_window = window_size / 2U;

    /* 计算搜索窗口起始和结束位置（考虑循环） */
    if (center_pos >= half_window) {
        start_pos = center_pos - half_window;
    } else {
        start_pos = buf_size - (half_window - center_pos);
    }

    end_pos = (center_pos + half_window) % buf_size;

    /* 在窗口内查找最大值 */
    if (start_pos < end_pos) {
        /* 正常情况：不跨越边界 */
        for (i = start_pos; i <= end_pos; i++) {
            if (buf[i] > peak_value) {
                peak_value = buf[i];
                peak_pos = i;
            }
        }
    } else {
        /* 跨越边界：分两段搜索 */
        for (i = start_pos; i < buf_size; i++) {
            if (buf[i] > peak_value) {
                peak_value = buf[i];
                peak_pos = i;
            }
        }
        for (i = 0U; i <= end_pos; i++) {
            if (buf[i] > peak_value) {
                peak_value = buf[i];
                peak_pos = i;
            }
        }
    }

    return peak_pos;
}

/**
 * @brief 获取过流保护引脚状态
 * @return 0: 安全（两个引脚都为低电平），1: 不安全（至少一个引脚为高电平）
 * @note 当两个引脚都为低电平时，表示硬件已恢复安全状态
 */
uint8_t ocd_get_ocp_pin_status(void)
{
    uint8_t positive_pin;
    uint8_t negative_pin;

    /* 读取正脉冲和负脉冲过流保护引脚状态 */
    positive_pin = gpio_read(OCP_POSITIVE_PIN_ID);
    negative_pin = gpio_read(OCP_NEGTIVE_PIN_ID);

    /* 两个引脚都为低电平时，返回0（安全） */
    if ((positive_pin == 0U) && (negative_pin == 0U)) {
        return 0U;
    }

    /* 至少一个引脚为高电平，返回1（不安全） */
    return 1U;
}

/**
 * @brief 硬件复位过流保护引脚
 * @return 0: 成功，-1: 失败
 * @note  复位后需要检查引脚状态确认是否真正安全
 */
int ocd_reset_ocp_hardware(void)
{
    /* 将复位引脚置高，触发硬件恢复 */
    gpio_write(OCP_RESET_PIN_ID, 1);

    return 0;
}

/**
 * @brief 设置过流检测电压阈值
 * @param channel 检测通道
 * @param voltage_mv 电压阈值（毫伏）
 * @return 0: 成功，-1: 失败
 */
int ocd_set_threshold(ocd_ch_t channel, uint16_t voltage_mv)
{
    uint16_t dac_value;
    HAL_StatusTypeDef status;

    if (channel >= 2U) {
        return -1;
    }

    if (g_initialized == 0U) {
        return -1;
    }

    /* 转换为DAC值 */
    dac_value = voltage_to_dac_value(voltage_mv);

    /* 设置DAC */
    if (channel == OCD_POSITIVE) {
        /* 通道1（ADC1）对应COMP3，COMP3负端使用DAC1 */
        status = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
        if (status == HAL_OK) {
            g_ocd_threshold_ch1 = voltage_mv;
            LOG_D("Set CH1 (ADC1/COMP3) threshold: %d mV (DAC1=%d)", voltage_mv, dac_value);
            return 0;
        } else {
            LOG_E("Failed to set DAC1: %d", status);
            return -1;
        }
    } else {
        /* 通道2（ADC2）对应COMP1，COMP1负端使用DAC3 */
        status = HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
        if (status == HAL_OK) {
            g_ocd_threshold_ch2 = voltage_mv;
            LOG_D("Set CH2 (ADC2/COMP1) threshold: %d mV (DAC3=%d)", voltage_mv, dac_value);
            return 0;
        } else {
            LOG_E("Failed to set DAC3: %d", status);
            return -1;
        }
    }
}

/**
 * @brief 获取过流检测电压阈值
 * @param channel 检测通道
 * @param voltage_mv 输出参数，存储电压阈值（毫伏）
 * @return 0: 成功，-1: 失败
 */
int ocd_get_threshold(ocd_ch_t channel, uint16_t *voltage_mv)
{
    if (channel >= 2U) {
        return -1;
    }

    if (voltage_mv == NULL) {
        return -1;
    }

    if (g_initialized == 0U) {
        return -1;
    }

    /* 返回存储的阈值 */
    if (channel == OCD_POSITIVE) {
        *voltage_mv = g_ocd_threshold_ch1;
    } else {
        *voltage_mv = g_ocd_threshold_ch2;
    }

    LOG_D("Get CH%d threshold: %d mV", channel, *voltage_mv);
    return 0;
}

/**
 * @brief ADC值转电压值
 * @param adc_value ADC寄存器值（12位，0-4095）
 * @return 电压值（毫伏）
 * @note 假设VREF = 3.3V = 3300mV
 */
static uint16_t adc_value_to_voltage(uint16_t adc_value)
{
    const uint16_t VREF_MV = 3300U;
    const uint16_t ADC_MAX = 4095U;
    uint32_t voltage_mv;

    /* 计算：电压 = (ADC值 / 4095) * VREF */
    voltage_mv = ((uint32_t)adc_value * VREF_MV) / ADC_MAX;

    return (uint16_t)voltage_mv;
}

/**
 * @brief 电压转DAC值
 * @param voltage_mv 电压值（毫伏）
 * @return DAC寄存器值（12位，0-4095）
 * @note 假设VREF = 3.3V = 3300mV
 */
static uint16_t voltage_to_dac_value(uint16_t voltage_mv)
{
    /* DAC参考电压：3.3V = 3300mV */
    const uint16_t VREF_MV = 3300U;
    const uint16_t DAC_MAX = 4095U;
    uint32_t dac_value;

    /* 计算：DAC值 = (电压 / VREF) * 4095 */
    dac_value = ((uint32_t)voltage_mv * DAC_MAX) / VREF_MV;

    /* 限制在有效范围内 */
    if (dac_value > DAC_MAX) {
        dac_value = DAC_MAX;
    }

    return (uint16_t)dac_value;
}

/**
  * @brief 比较器触发回调函数
  */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    if (hcomp->Instance == COMP1) {
        ocd_handle_ocp_trigger(OCD_NEGTIVE);
    } else if (hcomp->Instance == COMP3) {
        ocd_handle_ocp_trigger(OCD_POSITIVE);
    }
}

