#ifndef APP_OSAL_H
#define APP_OSAL_H

#include "LoRaMacUsr.h"

#define HardWare_taskID 0
#define LoraMAC_taskID 1
#define APP_taskID 2

extern LoRaMacAppPara_t g_appData;

/* Private macro -------------------------------------------------------------*/
//事件定义
#define APP_PERIOD_SEND     0x0001


void APP_Init(u8 task_id);
u16  APP_ProcessEvent( u8 task_id, u16 events );
void APP_ShowMoteID( u32 moteID );

#endif
