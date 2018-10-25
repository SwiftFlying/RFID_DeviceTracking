#ifndef __COM_H
#define __COM_H

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

#include "../app/data.h"
#include "../os/os.h"
#include "../device/device.h"
#include "aes.h"
#include "cmac.h"

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
void UART_Send(uint8_t * str,uint16_t count);
void UART_Read(uint8_t *ch);
void UART_Start(void);
void UART_Rx_IRQHandler(void);

/*********************************************************************************************
SPI
**********************************************************************************************/
void SPI_Send( uint8_t addr, uint8_t *buffer, uint8_t size );
void SPI_Read( uint8_t addr, uint8_t *buffer, uint8_t size );




/*********************************************************************************************
LORA
**********************************************************************************************/

typedef union
{
    uint8_t Value;
    struct
    {
        uint8_t Tx              : 1;
        uint8_t Rx              : 1;
        uint8_t RxData          : 1;
        uint8_t Multicast       : 1;
        uint8_t RxSlot          : 2;
        uint8_t LinkCheck       : 1;
        uint8_t JoinAccept      : 1;
    }PACKED Bits;
}PACKED LoRaMacEventFlags_t;


extern bool IsLoRaMacNetworkJoined;
void OnRadioTxDone( void );//Function to be executed on Radio Tx Done event
void OnRadioRxDone( uint8_t *payload, uint16_t size, int8_t rssi, int8_t snr ); //Function to be executed on Radio Rx Done event
void OnRadioTxTimeout( void ); //Function executed on Radio Tx Timeout event
void OnRadioRxError( void );//Function executed on Radio Rx error event
void OnRadioRxTimeout( void );  //Function executed on Radio Rx Timeout event
void OnRadioFhssChangeChannel( uint8_t currentChannel );
void OnRadioCadDone( bool channelActivityDetected );
void LoRaMacInit(void);
void LoRaMac_Send(uint8_t *fBuffer, uint16_t fBufferSize, bool confirmed);
#define LoRaMacMemCpy( src, dst, size ) memcpy1( dst, src, size )


#endif

