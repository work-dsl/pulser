/**
  ******************************************************************************
  * @file        : current_monitor.c
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 20xx-xx-xx
  * @brief       : 电流监控模块实现
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.初始版本
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "current_monitor.h"
#include "bsp_adc.h"
#include "errno-base.h"
#include "bsp_comp.h"
#include "bsp_dac.h"

#define  LOG_TAG             "current_monitor"
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
    uint16_t *buffer;          /**< DMA缓冲区指针 */
    uint16_t buf_size;         /**< 缓冲区大小（元素个数） */
    volatile uint16_t trigger_pos;  /**< 触发点在缓冲区中的位置 */
    volatile uint8_t trigger_flag;  /**< 触发标志 */
} channel_info_t;

/* Private define ------------------------------------------------------------*/

#define SEARCH_WINDOW_SIZE    (32U)  /**< 峰值搜索窗口大小（触发点前后32个点） */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 通道信息数组
 */
static channel_info_t g_channel_info[2];

/**
 * @brief 过流事件回调函数
 */
static ocp_info_cb_t g_ocp_callback = NULL;

/**
 * @brief 初始化标志
 */
static uint8_t g_initialized = 0U;

/* Exported variables  -------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint16_t get_dma_current_position(DMA_HandleTypeDef *hdma, uint16_t *buf, uint16_t buf_size);
static uint16_t find_peak_in_window(uint16_t *buf, uint16_t buf_size, uint16_t center_pos, uint16_t window_size);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化电流监控模块
 * @note 使用内部定义的缓冲区，无需外部传入
 */
int current_monitor_init(void)
{
    HAL_StatusTypeDef status;
    
    /* 使用内部定义的缓冲区 */
    g_channel_info[CURRENT_MONITOR_CH1].buffer = adc1_dma_buf;
    g_channel_info[CURRENT_MONITOR_CH1].buf_size = (uint16_t)sizeof(adc1_dma_buf) / sizeof(adc1_dma_buf[0]);
    g_channel_info[CURRENT_MONITOR_CH1].trigger_pos = 0U;
    g_channel_info[CURRENT_MONITOR_CH1].trigger_flag = 0U;
    
    g_channel_info[CURRENT_MONITOR_CH2].buffer = adc2_dma_buf;
    g_channel_info[CURRENT_MONITOR_CH2].buf_size = (uint16_t)sizeof(adc2_dma_buf) / sizeof(adc2_dma_buf[0]);
    g_channel_info[CURRENT_MONITOR_CH2].trigger_pos = 0U;
    g_channel_info[CURRENT_MONITOR_CH2].trigger_flag = 0U;
    
    g_initialized = 1U;
    
    LOG_D("Current monitor initialized: CH1 buf_size=%d, CH2 buf_size=%d", 
          g_channel_info[CURRENT_MONITOR_CH1].buf_size,
          g_channel_info[CURRENT_MONITOR_CH2].buf_size);
    
    return 0;
}

/**
 * @brief 启动ADC DMA循环采样
 */
int current_monitor_start(void)
{
    HAL_StatusTypeDef status;
    
    if (g_initialized == 0U) {
        return -ENODEV;
    }
    
    /* 启动ADC1 DMA循环采样 */
    status = HAL_ADC_Start_DMA(&hadc1, 
                                (uint32_t*)g_channel_info[CURRENT_MONITOR_CH1].buffer,
                                g_channel_info[CURRENT_MONITOR_CH1].buf_size);
    if (status != HAL_OK) {
        LOG_E("Failed to start ADC1 DMA: %d", status);
        return -EIO;
    }
    
    /* 启动ADC2 DMA循环采样 */
    status = HAL_ADC_Start_DMA(&hadc2,
                                (uint32_t*)g_channel_info[CURRENT_MONITOR_CH2].buffer,
                                g_channel_info[CURRENT_MONITOR_CH2].buf_size);
    if (status != HAL_OK) {
        LOG_E("Failed to start ADC2 DMA: %d", status);
        return -EIO;
    }
    
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
    
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1985);
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1985);
    
    status = HAL_COMP_Start(&hcomp1);
    if (status != HAL_OK)
    {
        LOG_E("Failed to start COMP1: %d", status);
        return -EIO;
    }
    
    status = HAL_COMP_Start(&hcomp3);
    if (status != HAL_OK)
    {
        LOG_E("Failed to start COMP1: %d", status);
        return -EIO;
    }
    
    LOG_D("ADC DMA started");
    return 0;
}

/**
 * @brief 停止ADC DMA循环采样
 */
void current_monitor_stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    LOG_D("ADC DMA stopped");
}

/**
 * @brief 设置过流事件回调函数
 */
void current_monitor_set_ocp_callback(ocp_info_cb_t callback)
{
    g_ocp_callback = callback;
}

/**
 * @brief 处理过流事件（在比较器中断中调用）
 * @details 过流发生后立即停止ADC采集，标记DMA位置
 */
void current_monitor_handle_ocp_trigger(current_monitor_ch_t channel)
{
    channel_info_t *ch_info;
    uint16_t dma_pos;
    
    if (channel >= 2U) {
        return;
    }
    
    ch_info = &g_channel_info[channel];
    
    /* 立即停止对应通道的ADC DMA采集，保持缓冲区数据不变 */
    if (channel == CURRENT_MONITOR_CH1) {
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
void current_monitor_process_ocp_event(void)
{
    uint8_t ch;
    channel_info_t *ch_info;
    ocp_info_t ocp_info;
    uint16_t peak_value;
    uint16_t peak_pos;
    
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
            peak_value = ch_info->buffer[peak_pos];
            
            /* 构建过流信息 */
            ocp_info.channel = (current_monitor_ch_t)ch;
            ocp_info.peak_value = peak_value;
            ocp_info.trigger_position = ch_info->trigger_pos;
            extern uint32_t HAL_GetTick(void);
            ocp_info.timestamp = HAL_GetTick();
            
            /* 调用回调函数上报 */
            if (g_ocp_callback != NULL) {
                g_ocp_callback(&ocp_info);
            }
            
            LOG_I("OCP processed: CH%d, peak=%d at pos=%d (from stopped buffer)", ch, peak_value, peak_pos);
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
  * @brief 比较器触发回调函数
  */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    if (hcomp->Instance == COMP1) {
        /* COMP1对应ADC2通道 */
        current_monitor_handle_ocp_trigger(CURRENT_MONITOR_CH2);
    } else if (hcomp->Instance == COMP3) {
        /* COMP3对应ADC1通道 */
        current_monitor_handle_ocp_trigger(CURRENT_MONITOR_CH1);
    }
}

