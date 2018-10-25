#include "../com.h"

extern UART_HandleTypeDef   UartHandle;



void UART_Start(void)
{
		__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);/* Enable the UART Data Register not empty Interrupt */
}

void UART_Send(uint8_t * str,uint16_t count)
{
	uint16_t i = 0 ;
	for(i = 0;i<count;i++)
	{
		USART1->TDR =  (uint8_t)(str[i]); 
		while((USART1->ISR & UART_FLAG_TC)!=UART_FLAG_TC );
	}	
}

uint8_t UART_RxBuf[UART_RxBuf_Size];
uint16_t UART_RxBuf_Top=0;
uint16_t UART_RxBuf_Rear=0;

uint8_t UART_RxBuf2[25];
uint16_t UART_RxBuf2_Top=0;
uint8_t UART_RxBuf2_Rear=0;


void UART_Rx_IRQHandler(void)
{
		 uint8_t byte;
		 byte= (uint8_t)(USART1->RDR );		 
		 if(byte==0xBB)
		 {
			 UART_RxBuf2_Top = 0;
			 memset1(UART_RxBuf2,0,25);//Çå¿ÕÊý¾Ý
			 UART_RxBuf2[UART_RxBuf2_Top++]=byte;
		 }	
		 else if(byte==0x7E&&UART_RxBuf2_Top==23)
		 {		 
			 UART_RxBuf[UART_RxBuf_Top]=UART_RxBuf2[18];
			 UART_RxBuf_Top=(UART_RxBuf_Top+1)%UART_RxBuf_Size;
			 UART_RxBuf[UART_RxBuf_Top]=UART_RxBuf2[19];
			 UART_RxBuf_Top=(UART_RxBuf_Top+1)%UART_RxBuf_Size;	 			 		 
		 }
		 else
		 {
			 UART_RxBuf2[UART_RxBuf2_Top++]=byte;	 
		 }	
}



