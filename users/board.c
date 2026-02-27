/**
  ******************************************************************************
  * @copyright   : Copyright To Hangzhou Dinova EP Technology Co.,Ltd
  * @file        : xxxx.c
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
/* Includes ------------------------------------------------------------------*/
#include "board.h"

#include "bsp_conf.h"
#include "bsp_gpio.h"
#include "bsp_hrtim.h"
#include "bsp_dma.h"
#include "bsp_sram.h"
#include "bsp_usart.h"
#include "bsp_adc.h"
#include "bsp_tim.h"
#include "bsp_comp.h"
#include "bsp_dac.h"
#include "bsp_iwdg.h"
#include "bsp_dwt.h"

#include "SEGGER_RTT.h"
#include "log.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables  -------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void rtt_output_handler(const char *msg, size_t len);

#if LOG_WITH_TIMESTAMP
/**
 * @brief 获取当前时间戳（毫秒）
 * @return 时间戳值（毫秒）
 * @note 使用DWT计数器实现高精度时间戳
 */
uint32_t log_time_now(void)
{
    extern uint32_t SystemCoreClock;
    double seconds;
    uint32_t result;
    
    seconds = bsp_dwt_get_seconds();
    result = (uint32_t)(seconds * 1000.0);
    
    return result;
}
#endif

/* Exported functions --------------------------------------------------------*/
/**
  * @brief
  * @param
  * @retval
  * @note
  */
void board_init(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();
      
    /* 调试代码初始化 */
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(0, "RTT", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    log_init();
    log_register_handler("rtt", rtt_output_handler);
    log_enable_handler("rtt");
    
    LOG_D("SYSCLK frequency is %d!", HAL_RCC_GetSysClockFreq());

    /* 检查复位源 */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        LOG_D("System reset by Software");
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        LOG_D("System reset by IWDG timeout");
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();  /* 清除复位标志 */

    /* Initialize all configured peripherals */
    bsp_gpio_init();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    bsp_dwt_init();
    bsp_iwdg_init();
    bsp_sram_init();
    bsp_uart_init();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
#if defined(SOC_SERIES_STM32F1)
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* NOJTAG: JTAG-DP Disabled and SW-DP Enabled */
#else
    __HAL_RCC_SYSCFG_CLK_ENABLE();
#endif
    __HAL_RCC_PWR_CLK_ENABLE();
    
#ifdef UCPD1
    /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
    */
    HAL_PWREx_DisableUCPDDeadBattery();
#endif
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {

    }
}

/**
 * @brief RTT输出处理器
 * @param msg 日志消息
 * @param len 消息长度
 * @note 使用WriteSkipNoLock因为log系统已经有锁保护，避免双重锁开销
 */
static void rtt_output_handler(const char *msg, size_t len)
{
    SEGGER_RTT_Write(0, msg, len);
}

/**
  Put a character to the stdout

  \param[in]   ch  Character to output
  \return          The character written, or -1 on write error.
*/
int stdout_putchar (int ch)
{
    SEGGER_RTT_PutChar(0, ch);
    return (-1);
}

/* Private functions ---------------------------------------------------------*/


