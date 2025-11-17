/**
  ******************************************************************************
  * @copyright   : Copyright To Hangzhou Dinova EP Technology Co.,Ltd
  * @file        : xxx.h
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
#ifndef __DATA_MGMT_H__
#define __DATA_MGMT_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/*
 1.发布版本
   软件发布版本号为：1
 2.完整版本
   嵌入式软件完整版本为：1.0.0.0
 3.版本命名规则
   软件的完整版本为：X.Y.Z.B(X.Y.Z.B为阿拉伯数字), 
   发布版本为V X.
      其中 X 表示重大增强类软件更新，
           Y 表示轻微增强类软件更新，
           Z 表示纠正类软件更新，
           B 表示软件构建，
           则软件完整版本为 X.Y.Z.B，
   软件发布版本为X，此时 X发生变化应进行许可事项变更，而Y、Z、B发生变化无需进行注册变更.
*/
/* Software Version Definitions */
#define SW_VERSION_X    "0"    /* X: 重大增强类软件更新 */
#define SW_VERSION_Y    "0"    /* Y: 轻微增强类软件更新 */
#define SW_VERSION_Z    "0"    /* Z: 纠正类软件更新 */
#define SW_VERSION_B    "0"    /* B: 软件构建号 */

/* Complete Version String - 通过组合各个版本号构建完整版本字符串 */
#define SW_VERSION_STRING   (SW_VERSION_X "." SW_VERSION_Y "." \
                             SW_VERSION_Z "." SW_VERSION_B)
                             

#define HW_VERSION_DEFAULT_STRING   ("0.0.0.0")
#define HW_VERSION_BUFSZ            (64)
#define SN_NUMBER_DEFAULT_STRING    ("SN0000000000")
#define SN_NUMBER_BUFSZ             (64)

#define BOARD_INFO_MAGIC_NUM        (0xFFDDFFDDFFDDFFDDULL)
#define BOARD_INFO_BASE_ADDR        (0x00000000)
/* Exported typedef ----------------------------------------------------------*/
typedef struct __attribute__((packed))
{
	uint64_t  magic;                     /**< 魔术码 */
    /* 模块硬件版本信息(ASCII码格式共13字节)：
    X：2字节；.字符：1字节；Y：2字节；.字符：1字节；Z：2字节；.字符：1字节；B：4字节*/
	char hw_version[HW_VERSION_BUFSZ];    
	char sn_number[SN_NUMBER_BUFSZ];    /* 模块SN码(ASCII码格式共12字节)：SNYYMMDDNNNN*/
}board_info_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported variable prototypes ----------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/
void data_mgmt_init(void);
const char* data_mgmt_get_sw_version(void);
const char* data_mgmt_get_hw_version(void);
const char* data_mgmt_get_sn_number(void);
int data_mgmt_set_hw_version(const char *string, uint16_t len);
int data_mgmt_set_sn_number(const char *string, uint16_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DATA_MGMT_H__ */

