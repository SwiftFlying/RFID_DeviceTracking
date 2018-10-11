/**
  ******************************************************************************
  * @file    stm32l0xx_hal_msp_template.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  *          This file template is located in the HAL folder and should be copied 
  *          to the user folder.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is eventually modified by the user.

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  __TIM2_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
	
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	  GPIO_InitTypeDef   GPIO_InitStructure;	
		__GPIOB_CLK_ENABLE();
		__USART1_CLK_ENABLE(); /* Enable USART1 clock */
		GPIO_InitStructure.Mode= GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull= GPIO_PULLUP;    
		GPIO_InitStructure.Speed= GPIO_SPEED_FAST;
		GPIO_InitStructure.Alternate = GPIO_AF0_USART1;
		GPIO_InitStructure.Pin= GPIO_PIN_6;//PB6-UART TX
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = GPIO_PIN_7;//PB7-UART RX 
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);		
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);//Configure the NVIC for UART1
		HAL_NVIC_EnableIRQ(USART1_IRQn);	
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
  HAL_NVIC_DisableIRQ(USART1_IRQn);	
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	  GPIO_InitTypeDef   GPIO_InitStruct;	
	  __GPIOA_CLK_ENABLE();
	  __SPI1_CLK_ENABLE();	

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH  ;
		GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
		GPIO_InitStruct.Pin       = GPIO_PIN_5;//PA5-SPI SCK 		
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_InitStruct.Pin       = GPIO_PIN_6;//PA6-SPI MISO
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_InitStruct.Pin       = GPIO_PIN_7;//PA7-SPI MOSI			
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_InitStruct.Pin       = GPIO_PIN_4;//PA4-SPI NSS pin: 
		GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
		HAL_NVIC_SetPriority(SPI1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);	
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  __HAL_RCC_SPI1_FORCE_RESET();
  __HAL_RCC_SPI1_RELEASE_RESET();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
  HAL_NVIC_DisableIRQ(SPI1_IRQn);	
}

