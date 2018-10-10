#include "cpu.h"
#include "sx1276.h"


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

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;	

#if (BUFFER_CACHE_DISABLE != 0)
	__HAL_FLASH_BUFFER_CACHE_DISABLE();// Configure Buffer cache 
#endif // BUFFER_CACHE_DISABLE 
#if (PREREAD_ENABLE != 0)
	__HAL_FLASH_PREREAD_BUFFER_ENABLE();//Configure Flash prefetch,  Flash preread  
#endif  // PREREAD_ENABLE 
#if (PREFETCH_ENABLE != 0)
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();// Configure Flash preread  
#endif // PREFETCH_ENABLE 
	HAL_InitTick(TICK_INT_PRIORITY);	// Use systick as time base source and configure 1ms tick (default clock after Reset is MSI) 

  GPIO_Init();	
/***************************************************************************************************************
RCC
**************************************************************************************************************/
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
	
/***************************************************************************************************************
RTC
**************************************************************************************************************/




/***************************************************************************************************************
systick
**************************************************************************************************************/
  if (SysTick_Config(SystemCoreClock/1000)) //SysTick配置函数，1 msec interrupts
    while (1);  // Capture error  
  NVIC_SetPriority(SysTick_IRQn, 0x0);//SysTick中断优先级设置

/***************************************************************************************************************
tim，100us 定时: 32MHz/3200=10000Hz
**************************************************************************************************************/
	TimHandle.Instance = TIM2;  /* Set TIMx instance */
	TimHandle.Init.Period = 3199;
	TimHandle.Init.Prescaler = 0;//uwPrescalerValue;
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&TimHandle);
	HAL_TIM_Base_Start_IT(&TimHandle);



/***************************************************************************************************************
EXTI
**************************************************************************************************************/
    //SX1276SetOpMode( RF_OPMODE_SLEEP );   //20160317

		//HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);/* Enable and set EXTI4_15 Interrupt to the lowest priority */
		//HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
		HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
		HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
		HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/***************************************************************************************************************
uart
**************************************************************************************************************/
	UartHandle.Instance        = USART1;
	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	
  if(UartHandle.Init.HwFlowCtl != UART_HWCONTROL_NONE)/* Check the parameters */  
    assert_param(IS_UART_HWFLOW_INSTANCE(&UartHandle->Instance));
  else/* Check the parameters */
    assert_param(IS_UART_INSTANCE(&UartHandle->Instance));
  if(UartHandle.State == HAL_UART_STATE_RESET)    /* Init the low level hardware : GPIO, CLOCK, CORTEX */
  {
		__USART1_CLK_ENABLE(); /* Enable USART1 clock */


		HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);//Configure the NVIC for UART1
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}		
  UartHandle.State = HAL_UART_STATE_BUSY;  
  __HAL_UART_DISABLE(&UartHandle);/* Disable the Peripheral */
  UART_SetConfig(&UartHandle);/* Set the UART Communication parameters */
  if (UartHandle.AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
    UART_AdvFeatureConfig(&UartHandle);
  /* In asynchronous mode, the following bits must be kept cleared: 
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  UartHandle.Instance->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN); 
  UartHandle.Instance->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN); 
    
 
  __HAL_UART_ENABLE(&UartHandle); /* Enable the Peripheral */
  UART_CheckIdleState(&UartHandle);  /* TEACK and/or REACK to check before moving &UartHandle->State to Ready */

	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_PE);		/* Enable the UART Parity Error Interrupt */
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_ERR);	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_DISABLE_IT(&UartHandle, UART_IT_RXNE);

	
	
/***************************************************************************************************************
SPI
**************************************************************************************************************/
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
    SpiHandle.Instance = SPI1;

    /* On STM32L0538-DISCO, EPD ID cannot be read then keep a common configuration */
    /* for EPD (SPI_DIRECTION_2LINES) */
    /* Note: To read a register a EPD, SPI_DIRECTION_1LINE should be set */
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



		__SPI1_CLK_ENABLE();	
		HAL_SPI_Init(&SpiHandle);
  }

	
/***************************************************************************************************************
I2C
**************************************************************************************************************/

	
	
/***************************************************************************************************************
SDIO
**************************************************************************************************************/

	
	
/***************************************************************************************************************
ADC
**************************************************************************************************************/

	
	
/***************************************************************************************************************
DAC
**************************************************************************************************************/

	
	
/***************************************************************************************************************
FPU
**************************************************************************************************************/

	
/***************************************************************************************************************
FMC
**************************************************************************************************************/

/***************************************************************************************************************
PWR
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
			HAL_IWDG_Start(&IWDGHandle);
*/
}




/***************************************************************************************************************
SYSTICK
**************************************************************************************************************/
uint32_t SysTick_Delay_ms_value;
extern uint16_t rf_tick;
extern uint16_t StopTime_tick;
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
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __TIM2_CLK_ENABLE();

  /* The used GPIO (LED2 port) will be configured in the main program through
  LED2 initialization method */

  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
		GPIO_InitStructure.Pin = GPIO_PIN_11;    //NRESET-PB11
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

		
		GPIO_InitStructure.Pin = (GPIO_PIN_2);//sx1279 board use active crystal, and PA2 control it's power supply
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);		
//EXTI	
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10;//DIO0 -- PB10, DIO1 -- PB2, DIO2 -- PB1, DIO3 -- PB0,
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
//UART			
		GPIO_InitStructure.Pin       = GPIO_PIN_6;/* UART TX GPIO pin configuration  */
		GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull      = GPIO_PULLUP;    //GPIO_NOPULL;
		GPIO_InitStructure.Speed     = GPIO_SPEED_FAST;
		GPIO_InitStructure.Alternate = GPIO_AF0_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = GPIO_PIN_7;   /* UART RX GPIO pin configuration  */
		GPIO_InitStructure.Alternate = GPIO_AF0_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
//SPI	
		GPIO_InitStructure.Pin = GPIO_PIN_5;     /* Configure SPI SCK */
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull  = GPIO_PULLDOWN;  //GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStructure.Alternate = GPIO_AF0_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = GPIO_PIN_7;/* Configure SPI MOSI */
		GPIO_InitStructure.Alternate = GPIO_AF0_SPI1;
		GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = GPIO_PIN_6;/* Configure SPI MISO */
		GPIO_InitStructure.Alternate = GPIO_AF0_SPI1;
		GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = GPIO_PIN_4; /* Configure SPI NSS pin: */
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
}
	

void GPIO_Write(void)
{
}

void GPIO_Read(void)
{
}



/***************************************************************************************************************
EXTI
**************************************************************************************************************/
void EXTI0_1_IRQHandler( void )
{
  //PB0 -- DIO3
	/* EXTI line interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
      SX1276OnDio3Irq();
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	}

  //PB1 -- DIO2
	/* EXTI line interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
	{
      SX1276OnDio2Irq();
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	}
}

void EXTI2_3_IRQHandler( void )
{
  //PB2 -- DIO1
	/* EXTI line interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
	{
      SX1276OnDio1Irq();
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
	}
}

void EXTI4_15_IRQHandler( void )
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
	{
		HAL_GPIO_EXTI_Callback(GPIO_PIN_8);
		//halProcessKeyInterrupt();
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
	}

  //PB10 -- DIO0
	/* EXTI line interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
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
void UART_Rx_IRQHandler(void)
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




void USART1_IRQHandler(void)
{
  UART_HandleTypeDef *huart = &UartHandle;
  /* UART parity error interrupt occurred ------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE) != RESET))
  {
          __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);

          huart->ErrorCode |= HAL_UART_ERROR_PE;
          /* Set the UART state ready to be able to start again the process */
          huart->State = HAL_UART_STATE_READY;
  }

  /* UART frame error interrupt occured --------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
  {
          __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);

          huart->ErrorCode |= HAL_UART_ERROR_FE;
          /* Set the UART state ready to be able to start again the process */
          huart->State = HAL_UART_STATE_READY;
  }

  /* UART noise error interrupt occured --------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
  {
          __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);

          huart->ErrorCode |= HAL_UART_ERROR_NE;
          /* Set the UART state ready to be able to start again the process */
          huart->State = HAL_UART_STATE_READY;
  }

  /* UART Over-Run interrupt occured -----------------------------------------*/
  if((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
  {
          __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);

          huart->ErrorCode |= HAL_UART_ERROR_ORE;
          /* Set the UART state ready to be able to start again the process */
          huart->State = HAL_UART_STATE_READY;
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
          huart->State = HAL_UART_STATE_READY;
          HAL_UARTEx_WakeupCallback(huart);
  }

  /* UART in mode Receiver ---------------------------------------------------*/	
  if((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET))
  {
     UART_Rx_IRQHandler();
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

				SpiHandle.State = HAL_SPI_STATE_BUSY;
				__HAL_SPI_DISABLE(&SpiHandle); /* Disable the SPI Peripheral Clock */
				HAL_SPI_MspDeInit(&SpiHandle);  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
				SpiHandle.ErrorCode = HAL_SPI_ERROR_NONE;
				SpiHandle.State = HAL_SPI_STATE_RESET;
				__HAL_UNLOCK(&SpiHandle);		  /* Release Lock */
		
				__SPI1_CLK_DISABLE();
		
		
				GPIO_InitStruct.Pin = GPIO_PIN_5;   //SPIx_SCK_PIN;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull  = GPIO_NOPULL;  //GPIO_PULLUP;
				GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
				GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				GPIO_InitStruct.Pin = GPIO_PIN_7; //Configure SPI MOSI
				GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				GPIO_InitStruct.Pull  = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				GPIO_InitStruct.Pin = GPIO_PIN_6;//Configure SPI MISO
				GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				GPIO_InitStruct.Pin = GPIO_PIN_4;				//Configure SPI NSS pin: 
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      	HAL_TIM_Base_DeInit(&TimHandle);
		
				GPIO_InitStruct.Pin = (GPIO_PIN_0 |GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 );
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				
				GPIO_InitStruct.Pin = GPIO_PIN_11;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);//PB11控制sx芯片的reset引脚
				//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
				GPIO_InitStruct.Pin = GPIO_PIN_2;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); //PA2控制sx芯片的有源晶振
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);			
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
		case 9: //恢复
			
			
		break;
		
	}
			
}




