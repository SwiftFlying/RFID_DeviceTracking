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





#define SetTxStatus() {transmitStatus = 0x01;}
#define ClrTxStatus() {transmitStatus = 0x00;}
#define TxReady() (transmitStatus==0x01)
#define COMMUNICATION_SOF_ERROR 0x01
#define COMMUNICATION_SUCCESS 0x02

#define SetPwmScale(value) {pwm_scale=value;}
#define GetPwmScale() ( pwm_scale )

extern uint8_t CmdProcess(uint8_t *pFrameIn);
extern uint8_t transmitStatus;
extern uint8_t transmitBuf[70];

extern volatile uint8_t pwm_scale;


//bsp_rcc****************************************************************************
void RCC_Init(void);


//bsp_rtc****************************************************************************
#ifndef TimerTime_t
typedef uint64_t TimerTime_t;
#endif

#define RTC_CLOCK_SOURCE_LSE
//#define RTC_CLOCK_SOURCE_LSI

#ifdef RTC_CLOCK_SOURCE_LSI
  #define RTC_ASYNCH_PREDIV    0x7F
  #define RTC_SYNCH_PREDIV     0x0130
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
  #define RTC_ASYNCH_PREDIV  0x7F
  #define RTC_SYNCH_PREDIV   0x00FF
#endif


void stm32l051_Init(void);
void RTC_Init( void );
void RtcStopTimer( void );
uint32_t RtcGetMinimumTimeout( void );
void RtcSetTimeout( uint32_t timeout );
TimerTime_t RtcGetTimerValue( void );
uint32_t RtcGetTimerElapsedTime( void );
void BlockLowPowerDuringTask ( bool Status );
void RtcEnterLowPowerStopMode( void );
void RtcRecoverMcuStatus( void );
void RtcDelayMs ( uint32_t delay );
void SystemPower_Config(void);
void RTC_AlarmConfig(void);
void RTC_IRQHandler(void);

//bsp_systick************************************************************************
void SysTick_Init(void);
void SysTick_Handler(void);
void SysTick_Delay_ms(uint32_t delay);

void SPI1_MspInit(SPI_HandleTypeDef *hspi);


//bsp_timer**************************************************************************
/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint64_t TimerTime_t;
#endif
#define HW_TIMER_TIME_BASE          100 //200£¬ Hardware Time base in us    

extern TIM_HandleTypeDef TimHandle;
extern volatile uint64_t TIM2_Tick;
extern volatile uint64_t TimeoutCntValue;
extern u32  osal_systemClock;// Milliseconds since last reboot



/*!
 * \brief Timer object description
 */


void TIM2_Init( void );//Initializes the timer,The timer is based on TIM2 with a 10uS time basis
void TIM2_IRQHandler( void );






//bsp_gpio**************************************************************************
void HalKeyInit( void );


//bsp_uart**************************************************************************
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)/* Size of Trasmission buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE/* Size of Reception buffer */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))


//bsp_spi**************************************************************************
/*!
 * SPI driver structure definition, for sx1276/1279
 */
struct Spi_s
{
    SPI_TypeDef *Spi;
    //Gpio_t Mosi;
    //Gpio_t Miso;
    //Gpio_t Sclk;
    //Gpio_t Nss;
};


/*!
 * SPI object type definition
 */
typedef struct Spi_s Spi_t;

extern SPI_HandleTypeDef SpiHandle;
#define SPIx_TIMEOUT_MAX              ((uint32_t)0x1000)

/*############################### SPI1 #######################################*/
#define SPIx                          SPI1
#define SPIx_CLK_ENABLE()             __SPI1_CLK_ENABLE()
#define SPIx_CLK_DISABLE()			  __SPI1_CLK_DISABLE()
#define SPIx_GPIO_PORT                GPIOA                      /* GPIOA */
#define SPIx_AF                       GPIO_AF0_SPI1

#define SPIx_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE()
#define SPIx_GPIO_CLK_DISABLE()       __GPIOA_CLK_DISABLE()

#define SPIx_SCK_PIN                  GPIO_PIN_5                 /* PA.05 */
#define SPIx_MISO_PIN                 GPIO_PIN_6                 /* PA.06 */
#define SPIx_MOSI_PIN                 GPIO_PIN_7                 /* PA.07 */
#define SPIx_NSS_PIN                  GPIO_PIN_4



void SPI1_Init(void);
void SPI1_DeInit(void);





//bsp_sdio**************************************************************************

//bsp_adc**************************************************************************
//bsp_dac**************************************************************************
//bsp_fpu**************************************************************************
//bsp_fmc**************************************************************************

//bsp_pwr**************************************************************************
//bsp_dog**************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
void IWDG_Init(void);
void IWDG_Feed(void);
#ifdef __cplusplus
}
#endif



/*!
 * Define indicating if an external IO expander is to be used
 */
#define BOARD_IOE_EXT

/*!
 * NULL definition
 */
#ifndef NULL
    #define NULL                                    ( ( void * )0 )
#endif

/*!
 * Generic definition
 */
//#ifndef SUCCESS
//#define SUCCESS                                     1
//#endif

#ifndef FAIL
#define FAIL                                        0
#endif



/*!
 * Random seed generated using the MCU Unique ID
 */
/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1     ( 0x1FF80050 )
#define         ID2     ( 0x1FF80054 )
#define         ID3     ( 0x1FF80064 )
#define RAND_SEED   ((*(uint32_t*)ID1)^(*(uint32_t*)ID2)^(*(uint32_t*)ID3))

/*!
 * Board IO Extender pins definitions
 */
#define IRQ_MPL3115                                 IOE_0
#define IRQ_MAG3110                                 IOE_1
#define GPS_POWER_ON                                IOE_2
#define RADIO_PUSH_BUTTON                           IOE_3
#define BOARD_POWER_DOWN                            IOE_4
#define SPARE_IO_EXT_5                              IOE_5
#define SPARE_IO_EXT_6                              IOE_6
#define SPARE_IO_EXT_7                              IOE_7
#define N_IRQ_SX9500                                IOE_8
#define IRQ_1_MMA8451                               IOE_9
#define IRQ_2_MMA8451                               IOE_10
#define TX_EN_SX9500                                IOE_11
#define LED_1                                       IOE_12
#define LED_2                                       IOE_13
#define LED_3                                       IOE_14
#define LED_4                                       IOE_15

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PB_10

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5
#define RADIO_NSS                                   PA_4

#define RADIO_DIO_0                                 PB_11
#define RADIO_DIO_1                                 PC_13
#define RADIO_DIO_2                                 PB_9
#define RADIO_DIO_3                                 PB_4
#define RADIO_DIO_4                                 PB_3
#define RADIO_DIO_5                                 PA_15

#define RADIO_ANT_SWITCH_HF                         PA_0
#define RADIO_ANT_SWITCH_LF                         PA_1

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define USB_DM                                      PA_11
#define USB_DP                                      PA_12

#define I2C_SCL                                     PB_6
#define I2C_SDA                                     PB_7

#define BOOT_1                                      PB_2

#define GPS_PPS                                     PB_1
#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define DC_DC_EN                                    PB_8
#define BAT_LEVEL                                   PB_0
#define WKUP1                                       PA_8
#define USB_ON                                      PA_2

#define RF_RXTX                                     PA_3

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

#define TEST_POINT1                                 PB_12
#define TEST_POINT2                                 PB_13
#define TEST_POINT3                                 PB_14
#define TEST_POINT4                                 PB_15

#define PIN_NC                                      PB_5

/*!
 * Board GPIO pin names
 */
typedef enum
{
    MCU_PINS,
    IOE_PINS,

    // Not connected
    NC = (int)0xFFFFFFFF
}PinNames;

/*!
 * Structure for the GPIO
 */
typedef struct
{
    PinNames  pin;
    uint16_t pinIndex;
    void *port;
    uint16_t portIndex;
}Gpio_t;


/*!
 * LED GPIO pins objects
 */

extern Gpio_t Led1;
extern Gpio_t Led2;
extern Gpio_t Led3;
extern Gpio_t Led4;
uint8_t BoardMeasureBatterieLevel( void );
void SpiInit( Spi_t *obj, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss );
void BoardGetUniqueId( uint8_t *id );






/*!
 * \brief  Find First Set
 *         This function identifies the least significant index or position of the
 *         bits set to one in the word
 *
 * \param [in]  value  Value to find least significant index
 * \retval bitIndex    Index of least significat bit at one
 */
 #if 0
__STATIC_INLINE uint8_t __ffs( uint32_t value )
{
    return( uint32_t )( 32 - __CLZ( value & ( -value ) ) );
}
#endif




#endif // __BSP_CONFIG_H__
