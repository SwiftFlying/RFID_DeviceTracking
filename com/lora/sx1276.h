#ifndef __SX1276_H__
#define __SX1276_H__

#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,  //1
}RadioModems_t;
typedef enum
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
}RadioState_t;

extern RadioState_t SX1276_State;
extern RadioModems_t SX1276_Modem;
extern uint32_t SX1276_Channel;
extern int8_t   SX1276_Power;
extern uint32_t SX1276_Bandwidth;
extern uint32_t SX1276_Datarate;
extern bool     SX1276_LowDatarateOptimize;
extern uint8_t  SX1276_Coderate;
extern uint16_t SX1276_PreambleLen;
extern bool     SX1276_FixLen;
extern uint8_t  SX1276_PayloadLen;
extern bool     SX1276_CrcOn;
extern bool     SX1276_FreqHopOn;
extern uint8_t  SX1276_HopPeriod;
extern bool     SX1276_IqInverted;
extern bool     SX1276_RxContinuous;
extern uint32_t SX1276_TxTimeout;
extern int8_t   SX1276_Payload_SnrValue;
extern int8_t   SX1276_Payload_RssiValue;
extern uint8_t  SX1276_Payload_Size;
extern uint8_t  SX1276_RxTx;

#define RADIO_WAKEUP_TIME   1000 // [us] Radio wakeup time from SLEEP mode
extern uint8_t work_frequence;
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625


void SX1276_Init(void);
void SX1276_RxConfig( RadioModems_t modem, uint32_t freq, uint32_t bandwidth,uint32_t datarate, uint8_t coderate,uint32_t bandwidthAfc, uint16_t preambleLen,uint16_t symbTimeout, bool fixLen,uint8_t payloadLen,bool crcOn, bool freqHopOn, uint8_t hopPeriod,bool iqInverted, bool rxContinuous,uint32_t timeout );
void SX1276_TxConfig( RadioModems_t modem, uint32_t freq, int8_t power, uint32_t fdev,uint32_t bandwidth, uint32_t datarate,uint8_t coderate, uint16_t preambleLen,bool fixLen, bool crcOn, bool freqHopOn,uint8_t hopPeriod, bool iqInverted, uint32_t timeout,uint64_t *GetTimeOnAir,uint8_t pktLen );
void SX1276_Send(uint8_t *buffer, uint8_t size);

void LoRaMac_setlowPowerMode(uint8_t enable);
void SX1276SetModem( RadioModems_t modem );
bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq );
void SX1276SetSleep( void );
void SX1276SetStby( void );
void SX1276StartCad( void );
int8_t SX1276ReadRssi( RadioModems_t modem );
void SX1276SetOpMode( uint8_t opMode );
uint32_t SX1276Random( void ) ;
void SX1276OnTimeoutIrq( void );
void SX1276OnDio0Irq( void );
void SX1276OnDio1Irq( void );
void SX1276OnDio2Irq( void );
void SX1276OnDio3Irq( void );
void SX1276OnDio4Irq( void );
void SX1276OnDio5Irq( void );




#endif // __SX1276_H__
