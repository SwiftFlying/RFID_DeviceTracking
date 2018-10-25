#include "bsp_Config.h"
#include "stm32l0xx_hal_rtc_ex.h"
#include "app.h"
#include "../com/com.h"


/**
 * Main application entry point.
 */
int main( void )
{
#if (BUFFER_CACHE_DISABLE != 0)
	__HAL_FLASH_BUFFER_CACHE_DISABLE();/* Configure Buffer cache */ 
#endif /* BUFFER_CACHE_DISABLE */
#if (PREREAD_ENABLE != 0)
	__HAL_FLASH_PREREAD_BUFFER_ENABLE();/* Configure Flash prefetch,  Flash preread */ 
#endif /* PREREAD_ENABLE */
#if (PREFETCH_ENABLE != 0)
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();/* Configure Flash preread */ 
#endif /* PREFETCH_ENABLE */
	HAL_InitTick(TICK_INT_PRIORITY);	/* Use systick as time base source and configure 1ms tick (default clock after Reset is MSI) */

  stm32l051_Init();
	LoRaMacInit();
  UART_Start();
  RFID_Init();
  while(1)
  {
	   RFID_Task();
		 TimerTask();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
