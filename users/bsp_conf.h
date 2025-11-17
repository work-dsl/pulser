/**
  ******************************************************************************
  * @file        : bsp_conf.h
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 2025-10-16
  * @brief       : BSP configuration file
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.Initial version
  ******************************************************************************
  */
#ifndef __BSP_CONF_H__
#define __BSP_CONF_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/
/* 芯片选择 */
#define SOC_SERIES_STM32G4

#if defined(SOC_SERIES_STM32F1)
    #include "stm32f1xx.h"
#elif defined(SOC_SERIES_STM32F4)
    #include "stm32f4xx.h"
#elif defined(SOC_SERIES_STM32G4)
    #include "stm32g4xx.h"
#else
#error "Please select first the soc series used in your application!"    
#endif

#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_exti.h"

/* 外设宏定义 */
/* 内部 flash 宏定义 */
#if defined(SOC_SERIES_STM32F1)
    #define STM32_FLASH_PAGE_NUM        (128UL)      /* F1系列总页数（根据实际芯片修改） */
    #define STM32_FLASH_USE_NUM         (16)         /* 使用的最后的 page/sector 数量 */
    #define STM32_FLASH_START_ADDR      (FLASH_BASE + (STM32_FLASH_PAGE_NUM - STM32_FLASH_USE_NUM) * FLASH_PAGE_SIZE)
    #define STM32_FLASH_END_ADDR        (FLASH_BASE + (STM32_FLASH_PAGE_NUM * FLASH_PAGE_SIZE) - 1)
    #define STM32_FLASH_ERASE_SIZE      FLASH_PAGE_SIZE
    #define STM32_FLASH_WRITE_UNIT      2            /* F1按半字（2字节）编程 */

#elif defined(SOC_SERIES_STM32F4)
    #define STM32_FLASH_SECTOR_NUM      (12UL)       /* F4系列总扇区数（根据实际芯片修改） */
    #define STM32_FLASH_USE_SECTORS     (2)          /* 使用的最后的扇区数量 */
    #define STM32_FLASH_START_ADDR      (0x080E0000) /* Sector 11起始地址（根据实际修改） */
    #define STM32_FLASH_END_ADDR        (0x080FFFFF) /* Flash末尾地址 */
    #define STM32_FLASH_ERASE_SIZE      (128*1024)   /* 最小擦除单元（扇区大小） */
    #define STM32_FLASH_WRITE_UNIT      4            /* F4按字（4字节）编程 */

#elif defined(SOC_SERIES_STM32G4)
    #define STM32_FLASH_PAGE_NUM        (64UL)       /* G4系列总页数（根据实际芯片修改） */
    #define STM32_FLASH_USE_NUM         (16)         /* 使用的最后的 page/sector 数量 */
    #define STM32_FLASH_START_ADDR      (FLASH_BASE + (STM32_FLASH_PAGE_NUM - STM32_FLASH_USE_NUM) * FLASH_PAGE_SIZE)
    #define STM32_FLASH_END_ADDR        (FLASH_BASE + (STM32_FLASH_PAGE_NUM * FLASH_PAGE_SIZE) - 1)
    #define STM32_FLASH_ERASE_SIZE      FLASH_PAGE_SIZE
    #define STM32_FLASH_WRITE_UNIT      8            /* G4按双字（8字节）编程 */
#else
    #error "Please define STM32 Flash configuration for your MCU series!"
#endif

/* ============================================================================
 * GPIO 引脚配置
 * ============================================================================
 */
#define LED_PIN_ID                      (25)    /* PB9 */

#define OCP_POSITIVE_PIN_ID             (104)   /* PG8,正脉冲过流保护引脚 */
#define OCP_NEGTIVE_PIN_ID              (103)   /* PG7,负脉冲过流保护引脚 */
#define OCP_RESET_PIN_ID                (105)   /* PG9,过流保护复位引脚 */

/* ============================================================================
 * UART使能配置
 * ============================================================================
 */
#define BSP_USING_UART1
#define BSP_UART1_RX_USING_DMA
#define BSP_UART1_TX_USING_DMA
#define BSP_USING_UART2
#define BSP_UART2_RX_USING_DMA
#define BSP_UART2_TX_USING_DMA

/* ============================================================================
 * USART1 配置
 * ============================================================================
 */
#ifdef BSP_USING_UART1
    #define BSP_UART1_TX_PIN                GPIO_PIN_4
    #define BSP_UART1_TX_PORT               GPIOC
    #define BSP_UART1_TX_AF                 GPIO_AF7_USART1
    #define BSP_UART1_RX_PIN                GPIO_PIN_5
    #define BSP_UART1_RX_PORT               GPIOC
    #define BSP_UART1_RX_AF                 GPIO_AF7_USART1
    #define UART1_RX_BUF_SIZE               256
    #define UART1_TX_BUF_SIZE               256
    #define BSP_UART1_IRQ_PRIORITY          0
#ifdef BSP_UART1_RX_USING_DMA
    #define BSP_UART1_DMA_RX_INSTANCE       DMA1_Channel5
    #define BSP_UART1_DMA_RX_IRQn           DMA1_Channel5_IRQn
    #define UART1_DMA_RX_IRQHandler         DMA1_Channel5_IRQHandler
    #define UART1_RX_TEMP_BUF_SIZE          64
#endif
#ifdef BSP_UART1_TX_USING_DMA
    #define BSP_UART1_DMA_TX_INSTANCE       DMA1_Channel6
    #define BSP_UART1_DMA_TX_IRQn           DMA1_Channel6_IRQn
    #define UART1_DMA_TX_IRQHandler         DMA1_Channel6_IRQHandler
#endif
#endif

/* ============================================================================
 * USART2 配置
 * ============================================================================
 */
#ifdef BSP_USING_UART2
    #define BSP_UART2_TX_PIN                GPIO_PIN_2
    #define BSP_UART2_TX_PORT               GPIOA
    #define BSP_UART2_TX_AF                 GPIO_AF7_USART2
    #define BSP_UART2_RX_PIN                GPIO_PIN_3
    #define BSP_UART2_RX_PORT               GPIOA
    #define BSP_UART2_RX_AF                 GPIO_AF7_USART2
    #define UART2_RX_BUF_SIZE               256
    #define UART2_TX_BUF_SIZE               256
    #define BSP_UART2_IRQ_PRIORITY          0
#ifdef BSP_UART2_RX_USING_DMA
    #define BSP_UART2_DMA_RX_INSTANCE       DMA1_Channel7
    #define BSP_UART2_DMA_RX_IRQn           DMA1_Channel7_IRQn
    #define UART2_DMA_RX_IRQHandler         DMA1_Channel7_IRQHandler
    #define UART2_RX_TEMP_BUF_SIZE          64
#endif
#ifdef BSP_UART2_TX_USING_DMA
    #define BSP_UART2_DMA_TX_INSTANCE       DMA1_Channel8
    #define BSP_UART2_DMA_TX_IRQn           DMA1_Channel8_IRQn
    #define UART2_DMA_TX_IRQHandler         DMA1_Channel8_IRQHandler
#endif
#endif

/* ============================================================================
 * 硬件定时器配置 - 根据实际需要注释或取消注释
 * ============================================================================
 */
#define BSP_USING_TIM2
#define BSP_USING_TIM6
//#define BSP_USING_TIM7

/* ============================================================================
 * TIM2 配置
 * ============================================================================
 */
#ifdef BSP_USING_TIM2
    #define BSP_TIM2_RESOLUTION_NS      1000        /* 1μs分辨率 */
    #define BSP_TIM2_IRQ_PRIORITY       5
#endif

/* ============================================================================
 * TIM6 配置
 * ============================================================================
 */
#ifdef BSP_USING_TIM6
    #define BSP_TIM6_RESOLUTION_NS      1000        /* 1μs分辨率 */
    #define BSP_TIM6_IRQ_PRIORITY       6
#endif

/* ============================================================================
 * TIM7 配置
 * ============================================================================
 */
#ifdef BSP_USING_TIM7
    #define BSP_TIM7_RESOLUTION_NS      10000       /* 10μs分辨率 */
    #define BSP_TIM7_IRQ_PRIORITY       6
#endif

#define ECG_TR_GPIO_PORT                GPIOB
#define ECG_TR_GPIO_PIN                 GPIO_PIN_6
#define ECG_TR_GPIO_AF                  GPIO_AF2_TIM4
#define ECG_TIMx                        TIM4
#define ECG_TIMx_IRQn                   TIM4_IRQn
#define ECG_TIMx_CH                     TIM_CHANNEL_1

/* Exported typedef ----------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variable prototypes ----------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __BSP_CONF_H__ */

