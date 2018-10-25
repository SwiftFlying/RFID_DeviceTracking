#include "cpu.h"
#include "sx1276.h"
#include "algorithm.h"


UART_HandleTypeDef   UartHandle;
SPI_HandleTypeDef SpiHandle;
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef    Input_Handle;
IWDG_HandleTypeDef   IWDGHandle;

__IO uint32_t uwLsiFreq = 0;
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;



void CPU_Init(void)
{
#if (BUFFER_CACHE_DISABLE != 0)
	__HAL_FLASH_BUFFER_CACHE_DISABLE();// Configure Buffer cache 
#endif // BUFFER_CACHE_DISABLE 
#if (PREREAD_ENABLE != 0)
	__HAL_FLASH_PREREAD_BUFFER_ENABLE();//Configure Flash prefetch,  Flash preread  
#endif  // PREREAD_ENABLE 
#if (PREFETCH_ENABLE != 0)
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();// Configure Flash preread  
#endif // PREFETCH_ENABLE 

	RCC_Init();
	SysTick_Init();
	Tim_Init();
  GPIO_Init();
  EXTI_Init();	
  UART_Init();
  SPI_Init();	
}


/***************************************************************************************************************
RCC
**************************************************************************************************************/
void RCC_Init(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;	
  __PWR_CLK_ENABLE();   // Enable Power Control clock 
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // The voltage scaling allows optimizing the power consumption when the device is clocked below the maximum system frequency, to update the voltage scaling value regarding system frequency refer to product datasheet

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;   
  RCC_OscInitStruct.HSICalibrationValue = 0x10;//RC校验
  HAL_RCC_OscConfig(&RCC_OscInitStruct);                    //PLLCLK=HSI*4/2

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //SYSCLK=PLLCLK
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        //AHBCLK=SYSCLK
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         //APB1CLK=AHBCLK
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         //APB2CLK=AHBCLK
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);	
}

/***************************************************************************************************************
SYSTICK
**************************************************************************************************************/
uint32_t SysTick_Delay_ms_value;
extern uint16_t rf_tick;
extern uint16_t StopTime_tick;
void SysTick_Init(void)
{
	HAL_InitTick(TICK_INT_PRIORITY);	// Use systick as time base source and configure 1ms tick (default clock after Reset is MSI) 	
  if (SysTick_Config(SystemCoreClock/1000)) //SysTick配置函数，1 msec interrupts
    while (1);  // Capture error  
  NVIC_SetPriority(SysTick_IRQn, 0x0);//SysTick中断优先级设置	
}
void SysTick_Handler(void)
{
	rf_tick++;
	StopTime_tick++;
  HAL_IncTick();
	if(SysTick_Delay_ms_value!=0)
		SysTick_Delay_ms_value--;
}
void SysTick_Delay_ms(uint32_t delay)
{
	SysTick_Delay_ms_value=delay;
	while(SysTick_Delay_ms_value>0);
}


/***************************************************************************************************************
TIM
**************************************************************************************************************/
uint32_t uwPrescalerValue = 0;


void Tim_Init(void) //100us 定时: 32MHz/3200=10000Hz
{
	TimHandle.Instance = TIM2;  /* Set TIMx instance */
	TimHandle.Init.Period = 3199;
	TimHandle.Init.Prescaler = 0;//uwPrescalerValue;
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&TimHandle);
	HAL_TIM_Base_Start_IT(&TimHandle);
}
/***************************************************************************************************************
GPIO
**************************************************************************************************************/
void GPIO_Init(void)
{
	  GPIO_InitTypeDef   GPIO_InitStructure;	
	  __GPIOA_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();
//OUT
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	
		GPIO_InitStructure.Pin = GPIO_PIN_11;//PB11,
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
		GPIO_InitStructure.Pin = (GPIO_PIN_2);//PA2, sx1279 board use active crystal, and PA2 control it's power supply
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);		
}

void EXTI_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;	
	__GPIOB_CLK_ENABLE();	

	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10;//PB10-DIO0, PB2-DIO1, PB1-DIO2, PB0-DIO3
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.Pin = GPIO_PIN_5 ;//PB5,
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);		
	
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);	
}
void EXTI0_1_IRQHandler( void )
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)//PB0-DIO3
	{
      SX1276OnDio3Irq();
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)//PB1-DIO2
	{
      SX1276OnDio2Irq();
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	}
}

void EXTI2_3_IRQHandler( void )
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)//PB2-DIO1
	{
      SX1276OnDio1Irq();
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
	}
}

void EXTI4_15_IRQHandler( void )
{
	uint8_t a[3]={0x1,0x2,0x3};
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)//PB5-WAKE UP
	{
		UART_Send(a,3);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
	}		
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
	{
		HAL_GPIO_EXTI_Callback(GPIO_PIN_8);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)//PB10 -- DIO0
	{
      SX1276OnDio0Irq();
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
	}
}


/***************************************************************************************************************
UART
**************************************************************************************************************/
uint8_t UART_TxBuf[UART_TxBuf_Size];/* 发送FIFO *///全局变量在源文件中定义
uint16_t UART_TxBuf_Top=0;		/* 发送FIFO写指针 */
uint16_t UART_TxBuf_Rear=0;		/* 发送FIFO读指针 */
uint8_t UART_RxBuf[UART_RxBuf_Size];
uint16_t UART_RxBuf_Top=0;
uint16_t UART_RxBuf_Rear=0;
uint8_t UART_RxBuf2[25];
uint16_t UART_RxBuf2_Top=0;
uint8_t UART_RxBuf2_Rear=0;

void UART_Init(void)
{
	UartHandle.Instance        = USART1;
	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	
  HAL_UART_Init(&UartHandle);
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_PE);		/* Enable the UART Parity Error Interrupt */
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_ERR);	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_DISABLE_IT(&UartHandle, UART_IT_RXNE);
}

void UART_Start(void)
{
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);/* Enable the UART Data Register not empty Interrupt */	
}
void UART_Send(uint8_t * buf,uint16_t count)
{
	uint16_t i;	
	for(i = 0;i<count;i++)
	{
		USART1->TDR =  (uint8_t)(buf[i]); 
		while((USART1->ISR & UART_FLAG_TC)!=UART_FLAG_TC);
	}	
}
void USART1_IRQHandler(void)
{
  UART_HandleTypeDef *huart = &UartHandle;
  /* UART parity error interrupt occurred ------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE) != RESET))
  {
		__HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);

		huart->ErrorCode |= HAL_UART_ERROR_PE;
		/* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
  }

  /* UART frame error interrupt occured --------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
  {
		__HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);

		huart->ErrorCode |= HAL_UART_ERROR_FE;
		/* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
  }

  /* UART noise error interrupt occured --------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
  {
		__HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);

		huart->ErrorCode |= HAL_UART_ERROR_NE;
		/* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
  }

  /* UART Over-Run interrupt occured -----------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
  {
		__HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);

		huart->ErrorCode |= HAL_UART_ERROR_ORE;
		/* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
  }

  /* Call UART Error Call back function if need be --------------------------*/
  if(huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
		HAL_UART_ErrorCallback(huart);
  }

  /* UART Wake Up interrupt occured ------------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_WUF) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_WUF) != RESET))
  {
		__HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);
		/* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
		HAL_UARTEx_WakeupCallback(huart);
  }

  /* UART in mode Receiver ---------------------------------------------------*/	
	if((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET))
  {
		uint8_t byte;	
		byte= (uint8_t)(USART1->RDR );		 
		if(byte==0xBB)
		{
		 UART_RxBuf2_Top = 0;
		 mem_Init(UART_RxBuf2,0,25);//清空数据
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
}

/***************************************************************************************************************
I2C
**************************************************************************************************************/




/***************************************************************************************************************
SPI
**************************************************************************************************************/
#define SPIx_TIMEOUT_MAX              ((uint32_t)0x1000)
#define SPI_NSS_LOW()            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SPI_NSS_HIGH()           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
void SPI_Init(void)
{
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    SpiHandle.Instance = SPI1;
    SpiHandle.Init.Mode               = SPI_MODE_MASTER;
    SpiHandle.Init.Direction          = SPI_DIRECTION_2LINES;
    SpiHandle.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.DataSize           = SPI_DATASIZE_8BIT;
    SpiHandle.Init.CLKPhase           = SPI_PHASE_1EDGE;//SPI_PHASE_2EDGE;
    SpiHandle.Init.CLKPolarity        = SPI_POLARITY_LOW;//SPI_POLARITY_HIGH;SPI_POLARITY_LOW
    SpiHandle.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS                = SPI_NSS_SOFT;
    SpiHandle.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial      = 7;
    SpiHandle.Init.TIMode             = SPI_TIMODE_DISABLED;
		HAL_SPI_Init(&SpiHandle);
  }	
}

//Writes multiple radio registers starting at address
void SPI_Send( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t write_addr = addr | 0x80;  //8位地址写入，第一位是1表示是写操作
	SPI_NSS_LOW();
	HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &write_addr, 1, SPIx_TIMEOUT_MAX);
	for(uint8_t i=0; i<size ; i++ )
			HAL_SPI_Transmit(&SpiHandle, (uint8_t*)(buffer+i), 1, SPIx_TIMEOUT_MAX);
	SPI_NSS_HIGH();
}

//Reads multiple radio registers starting at address
void SPI_Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t read_addr = addr & 0x7F;
	SPI_NSS_LOW();
  HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &read_addr, 1, SPIx_TIMEOUT_MAX);
	for(uint8_t i=0; i<size ; i++ )
	{	
		uint32_t readvalue = 0;
		uint32_t writevalue = 0xFFFFFFFF;
		HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SPIx_TIMEOUT_MAX);	
    buffer[i] = readvalue;
  }
	SPI_NSS_HIGH();
}



/***************************************************************************************************************
WATCH DOG
**************************************************************************************************************/
/***************************************************************************************************************
DOG
		##-2- Get the LSI frequency: TIM21 is used to measure the LSI frequency ###
		uwLsiFreq = GetLSIFrequency();
		
		##-3- Configure & Initialize the IWDG peripheral ######################################
		 Set counter reload value to obtain 250ms IWDG TimeOut.
			 IWDG counter clock Frequency = LsiFreq/32
			 Counter Reload Value = 250ms/IWDG counter clock period
														= 0.25s / (32/LsiFreq)
														= LsiFreq/(32 * 4)
														= LsiFreq/128 
*************************************************************************************************************
*/
void IWDG_Init(void)
{
	RCC_OscInitTypeDef oscinit={0};
	oscinit.OscillatorType=RCC_OSCILLATORTYPE_LSI;
	oscinit.LSIState=RCC_LSI_ON;
	oscinit.PLL.PLLState=RCC_PLL_NONE;
	HAL_RCC_OscConfig(&oscinit);
	IWDGHandle.Instance = IWDG;
	IWDGHandle.Init.Prescaler = IWDG_PRESCALER_8;
	IWDGHandle.Init.Reload =0xFFFF;
	IWDGHandle.Init.Window = IWDG_WINDOW_DISABLE;
	HAL_IWDG_Init(&IWDGHandle);
}
void IWDG_Feed(void)
{
  HAL_IWDG_Refresh(&IWDGHandle);
}


/***************************************************************************************************************
PWR
**************************************************************************************************************/
void PWR_LPMode(uint8_t mode)
{
	GPIO_InitTypeDef  GPIO_InitStruct;	
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct; 		
	switch (mode)
	{
		case 0: //降频
				__PWR_CLK_ENABLE();   // Enable Power Control clock 
				__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // The voltage scaling allows optimizing the power consumption when the device is clocked below the maximum system frequency, to update the voltage scaling value regarding system frequency refer to product datasheet

				RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
				RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
				RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
				RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF; 
		    RCC_OscInitStruct.MSIState = RCC_MSI_ON;		
				RCC_OscInitStruct.MSICalibrationValue = 0x10;//RC校验
		    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
				HAL_RCC_OscConfig(&RCC_OscInitStruct);                    //PLLCLK=HSI*4/2

				RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
				RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI; //SYSCLK=PLLCLK
				RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        //AHBCLK=SYSCLK
				RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         //APB1CLK=AHBCLK
				RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         //APB2CLK=AHBCLK
				HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);	
		break;
		case 1:
        __disable_irq( );

        //Radio.Sleep( );			

		    HAL_SPI_DeInit(&SpiHandle);
        HAL_UART_DeInit(&UartHandle);		
      	HAL_TIM_Base_DeInit(&TimHandle);
		
				GPIO_InitStruct.Pin = (GPIO_PIN_0 |GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 );
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				
				__GPIOB_CLK_DISABLE();  
				__GPIOA_CLK_DISABLE();   //20160317	

        __enable_irq( );

        /* Enter Stop Mode */
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

		break;
		case 2:
			
		break;
		case 3:
			
		break;
		case 4:
			
		break;
		case 5:
			
		break;
		case 6:
			
		break;
		case 7:
			
		break;
		case 8:
			
		break;

		
	}
			
}




