/**
  ******************************************************************************
  * @copyright: Copyright To Hangzhou Dinova EP Technology Co.,Ltd
  * @file     : xx.h
  * @author   : ZJY
  * @version  : V1.0
  * @date     : 20xx-xx-xx
  * @brief    : xxx
  *                   1.xx
  *                   2.xx
  *
  * @attention: None
  ******************************************************************************
  * @history  : 
  *      V1.0 : 1.xxx
  *
  *
  *     
  ******************************************************************************
  */
/*------------------------------ include -------------------------------------*/


/*------------------------------ Macro definition ----------------------------*/


/*------------------------------ typedef definition --------------------------*/

/**
 * @brief 过流保护事件回调函数类型
 */
typedef void (*ocp_event_cb_t)(void);

/*------------------------------ variable declarations -----------------------*/


/*------------------------------ function declarations -----------------------*/
void safety_init(void);
void safety_task(void);
void safety_perform_software_reset(void);
void safety_set_ocp_callback(ocp_event_cb_t callback);

/******************************* End Of File **********************************/

