/**
  ******************************************************************************
  * @copyright   : Copyright To Hangzhou Dinova EP Technology Co.,Ltd
  * @file        : data_mgmt.c
  * @author      : ZJY
  * @version     : V1.0
  * @data        : 20xx-xx-xx
  * @brief       : 
  * @attattention: None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.xxx
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "data_mgmt.h"
#include "mtd.h"
#include <string.h>
#include <stdio.h>

#define  LOG_TAG             "data_mgmt"
#define  LOG_LVL             4
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
static board_info_t g_board_info;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables  -------------------------------------------------------*/
extern struct mtd_info bsp_flash_info;

/* Private function prototypes -----------------------------------------------*/
static int board_info_flush(void);

/* Exported functions --------------------------------------------------------*/
void data_mgmt_init(void)
{
    size_t retlen;

    // Read the entire board_info structure from flash in one operation.
    mtd_read(&bsp_flash_info, BOARD_INFO_BASE_ADDR, sizeof(g_board_info), &retlen, (uint8_t*)&g_board_info);

    // If the magic number is incorrect or read failed, initialize with default values.
    if ((retlen != sizeof(g_board_info)) || (g_board_info.magic != BOARD_INFO_MAGIC_NUM))
    {
        g_board_info.magic = BOARD_INFO_MAGIC_NUM;
        strncpy(g_board_info.hw_version, HW_VERSION_DEFAULT_STRING, HW_VERSION_BUFSZ - 1);
        g_board_info.hw_version[HW_VERSION_BUFSZ - 1] = '\0'; // Ensure null termination

        strncpy(g_board_info.sn_number, SN_NUMBER_DEFAULT_STRING, SN_NUMBER_BUFSZ - 1);
        g_board_info.sn_number[SN_NUMBER_BUFSZ - 1] = '\0'; // Ensure null termination

        // Write the newly initialized default data to flash.
        board_info_flush();
        LOG_D("First write board infomation!");
    }
    
    LOG_D("SW_VERSION = %s!\r\n", data_mgmt_get_sw_version());
    LOG_D("HW_VERSION = %s!\r\n", data_mgmt_get_hw_version());
    LOG_D("SN_NUMBER = %s!\r\n", data_mgmt_get_sn_number());
}

const char* data_mgmt_get_sw_version(void)
{
    return SW_VERSION_STRING;
}

const char* data_mgmt_get_hw_version(void)
{
    return g_board_info.hw_version;
}

int data_mgmt_set_hw_version(const char *string, uint16_t len)
{
    // Validate input length to prevent buffer overflow.
    if (len >= HW_VERSION_BUFSZ) {
        return -1; // Error: Input string is too long.
    }

    // Copy the new string into the global struct.
    strncpy(g_board_info.hw_version, string, len);
    g_board_info.hw_version[len] = '\0'; // Ensure null termination

    // Write the entire updated struct to flash.
    return board_info_flush();
}

const char* data_mgmt_get_sn_number(void)
{
    // BUG FIX: Was returning hw_version, now correctly returns sn_number.
    return g_board_info.sn_number;
}

int data_mgmt_set_sn_number(const char *string, uint16_t len)
{
    // Validate input length to prevent buffer overflow.
    if (len >= SN_NUMBER_BUFSZ) {
        return -1; // Error: Input string is too long.
    }
    
    // Copy the new string into the global struct.
    strncpy(g_board_info.sn_number, string, len);
    g_board_info.sn_number[len] = '\0'; // Ensure null termination

    // Write the entire updated struct to flash.
    return board_info_flush();
}

/**
 * @brief Erases and writes the entire g_board_info struct to flash.
 * @return 0 on success, non-zero on failure.
 */
static int board_info_flush(void)
{
    struct erase_info info;
    size_t retlen;
    int ret;

    // Set up the erase operation for the entire info block.
    info.addr = BOARD_INFO_BASE_ADDR;
    info.len = sizeof(g_board_info);
    
    // Erase the relevant flash sector(s).
    ret = mtd_erase(&bsp_flash_info, &info);
    if (ret != 0) {
        // Handle erase error if necessary
        return ret;
    }

    // Write the entire updated struct back to flash.
    ret = mtd_write(&bsp_flash_info, BOARD_INFO_BASE_ADDR, sizeof(g_board_info), &retlen, (uint8_t*)&g_board_info);
    
    // Check if the write was successful
    if (ret != 0 || retlen != sizeof(g_board_info)) {
        // Handle write error if necessary
        return -1;
    }
    
    return 0;
}