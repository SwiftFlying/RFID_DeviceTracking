#include "app.h"
#include "os.h"
#include "algorithm.h"
/**
 * Main application entry point.
*/
int main( void )
{
  CPU_Init();
	LoRaMacInit();
  UART_Start();
  RFID_Init();
	PWR_LPMode(1);
	CPU_Init();
	RFID_Init();

  while(1)
  {
	  //RFID_Task();
		//TimerTask();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
