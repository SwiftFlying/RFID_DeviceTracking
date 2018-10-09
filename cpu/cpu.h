#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__


#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>


#include "stm32l051xx.h"
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"

#define SetPwmScale(value) {pwm_scale=value;}
#define GetPwmScale() ( pwm_scale )
extern volatile uint8_t pwm_scale;
extern SPI_HandleTypeDef SpiHandle;
extern TIM_HandleTypeDef TimHandle;
void CPU_Init(void);
void SysTick_Delay_ms(uint32_t delay);

#if defined(__CC_ARM) || defined(__GNUC__)
#define PACKED                                      __attribute__( ( __packed__ ) )
#elif defined( __ICCARM__ )
#define PACKED                                      __packed
#else
    #warning Not supported compiler type
#endif


/*********************************************************************************************
UART
**********************************************************************************************/
#define UART_RxBuf_Size 1000
extern uint8_t UART_RxBuf[UART_RxBuf_Size];
extern uint16_t UART_RxBuf_Top;
extern uint16_t UART_RxBuf_Rear;
#define UART_TxBuf_Size 1000
extern uint8_t UART_TxBuf[UART_TxBuf_Size];/* 发送FIFO *///全局变量在源文件中定义
extern uint16_t UART_TxBuf_Top;		/* 发送FIFO写指针 */
extern uint16_t UART_TxBuf_Rear;		/* 发送FIFO读指针 */
void UART_Send(uint8_t * str,uint16_t count);
void UART_Read(uint8_t *ch);
void UART_Start(void);
void UART_Rx_IRQHandler(void);
void UART_Tx_IRQHandler(void);
void UART_Tc_IRQHandler(void);

/*********************************************************************************************
I2C
**********************************************************************************************/
#define I2C_TxBuf_Size 1000
#define I2C_RxBuf_Size 1000
extern uint8_t I2C_TxBuf[I2C_TxBuf_Size];
extern uint8_t I2C_RxBuf[I2C_RxBuf_Size];

void I2C_Send(uint8_t DevAddr, uint8_t WriteAddr, uint8_t *data, uint8_t size);
void I2C_Read(uint8_t DevAddr, uint8_t ReadAddr, uint8_t *data, uint8_t size);
void I2C_WaitEepromStandbyState(uint8_t DevAddr);
void I2C_decimal_and_Integer(uint8_t cal_flag,long double *double_buffer,int *int_bufffer,u32 DOUBLE_ADDR,u32 LONGINT_ADDR);


/*********************************************************************************************
SPI
**********************************************************************************************/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)   
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))
extern uint32_t  SPITimeout;  
#define WIP_Flag                  0x01  /* Write In Progress (WIP) flag */
#define SPI5_CS_LOW()      {GPIOF->BSRRH=GPIO_Pin_6;}
#define SPI5_CS_HIGH()     {GPIOF->BSRRL=GPIO_Pin_6;}


#define SPI5_TxBuf_Size 1000
#define SPI5_RxBuf_Size 1000
extern uint8_t SPI5_TxBuf[SPI5_TxBuf_Size];
extern uint8_t SPI5_RxBuf[SPI5_RxBuf_Size];
u16 SPI_FLASH_SendHalfWord(u16 HalfWord);//发送半字
uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);
/*信息输出*/
#define FLASH_DEBUG_ON         1

#define FLASH_INFO(fmt,arg...)           printf("<<-FLASH-INFO->> "fmt"\n",##arg)
#define FLASH_ERROR(fmt,arg...)          printf("<<-FLASH-ERROR->> "fmt"\n",##arg)
#define FLASH_DEBUG(fmt,arg...)          do{\
                                          if(FLASH_DEBUG_ON)\
                                          printf("<<-FLASH-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

void SPI_Send( uint8_t addr, uint8_t *buffer, uint8_t size );
void SPI_Read( uint8_t addr, uint8_t *buffer, uint8_t size );
																					
																					
/*********************************************************************************************
WATCH DOG
**********************************************************************************************/
void IWDG_Feed(void);
																					
/*********************************************************************************************
PWR
**********************************************************************************************/																					
void PWR_LPMode(uint8_t mode);





#endif // __BSP_CONFIG_H__
