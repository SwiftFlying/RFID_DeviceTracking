#include "bsp_Config.h"
#include "../../../com/lora/sx1276.h"
#include "../../../com/com.h"


UART_HandleTypeDef   UartHandle;
TIM_HandleTypeDef    TimHandle;
IWDG_HandleTypeDef   IWDGHandle;
TIM_HandleTypeDef    Input_Handle;

__IO uint32_t uwLsiFreq = 0;
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;

void stm32l051_Init(void)
{
/***************************************************************************************************************
rcc，HSI=16MHz 、HSE=32MHz
**************************************************************************************************************/
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
	
	

/***************************************************************************************************************
rtc
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
gpio
**************************************************************************************************************/



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
		GPIO_InitTypeDef  GPIO_InitStruct;	
		__GPIOB_CLK_ENABLE();/* Enable GPIO TX/RX clock */
		__USART1_CLK_ENABLE(); /* Enable USART1 clock */

		GPIO_InitStruct.Pin       = GPIO_PIN_6;/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;    //GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_7;   /* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
spi
**************************************************************************************************************/
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
    SpiHandle.Instance = SPIx;

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

    SPI1_MspInit(&SpiHandle);
    HAL_SPI_Init(&SpiHandle);
  }

	
/***************************************************************************************************************
i2c
**************************************************************************************************************/

	
	
/***************************************************************************************************************
sdio
**************************************************************************************************************/

	
	
/***************************************************************************************************************
adc
**************************************************************************************************************/

	
	
/***************************************************************************************************************
dac
**************************************************************************************************************/

	
	
/***************************************************************************************************************
fpu
**************************************************************************************************************/

	
/***************************************************************************************************************
fmc
**************************************************************************************************************/

/***************************************************************************************************************
pwr
**************************************************************************************************************/


/***************************************************************************************************************
dog
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


SPI_HandleTypeDef SpiHandle;

/*********************************************************************
 * @fn		 SPI1_MspInit
 *
 * @brief 	 SPI MSP Init
 *
 * @param 	hspi: SPI handle
 *
 * @return	void
 */
void SPI1_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  SPIx_GPIO_CLK_ENABLE();

  /* Configure SPI SCK */
  GPIO_InitStruct.Pin = SPIx_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;  //GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = SPIx_AF;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI MOSI */
  GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
  GPIO_InitStruct.Alternate = SPIx_AF;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI MISO */
  GPIO_InitStruct.Pin = SPIx_MISO_PIN;
  GPIO_InitStruct.Alternate = SPIx_AF;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI NSS pin: */
  GPIO_InitStruct.Pin = SPIx_NSS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);
  /*** Configure the SPI peripheral ***/
  /* Enable SPI clock */
  SPIx_CLK_ENABLE();
}



/*********************************************************************
 * @fn		 SPI1_DeMspInit
 *
 * @brief 	 SPI MSP DeInit
 *
 * @param 	hspi: SPI handle
 *
 * @return	void
 */
void SPI1_DeMspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  //SPIx_GPIO_CLK_ENABLE();
  //SPIx_GPIO_CLK_DISABLE();  //20160317
  SPIx_CLK_DISABLE();
	
	  /* Configure SPI SCK */
  GPIO_InitStruct.Pin = SPIx_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;  //GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = SPIx_AF;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI MOSI */
  GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
  GPIO_InitStruct.Alternate = SPIx_AF;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI MISO */
  GPIO_InitStruct.Pin = SPIx_MISO_PIN;
  GPIO_InitStruct.Alternate = SPIx_AF;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

  /* Configure SPI NSS pin: */
  GPIO_InitStruct.Pin = SPIx_NSS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);
}

/*********************************************************************
 * @fn		 SPI1_DeInit
 *
 * @brief
 *
 * @param 	None
 *
 * @return	void
 */
 void SPI1_DeInit( void )
{
	HAL_SPI_DeInit(&SpiHandle);
	SPI1_DeMspInit(&SpiHandle);
}


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


void IWDG_Feed(void)
{
  HAL_IWDG_Refresh(&IWDGHandle);
}

