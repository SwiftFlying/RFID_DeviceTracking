#ifndef __COM_H
#define __COM_H

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>


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





#endif

