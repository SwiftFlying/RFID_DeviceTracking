#include "sx1276.h"
#include "os.h"


#define         ID1     ( 0x1FF80050 )
#define         ID2     ( 0x1FF80054 )
#define         ID3     ( 0x1FF80064 )
#define RAND_SEED   ((*(uint32_t*)ID1)^(*(uint32_t*)ID2)^(*(uint32_t*)ID3))


#define RF_MID_BAND_THRESH                          525000000

#define LORA_MAC_PUBLIC_SYNCWORD    0x34//Syncword for Public LoRa networks
#define LORA_MAC_PRIVATE_SYNCWORD   0x12//Syncword for Private LoRa networks
#define debug_rssi 1

static bool RadioIsActive = false;  //Flag used to set the RF switch control pins in low power mode when the radio is not active.


RadioState_t SX1276_State;
RadioModems_t SX1276_Modem;
uint32_t SX1276_Channel;
int8_t   SX1276_Power;
uint32_t SX1276_Bandwidth;
uint32_t SX1276_Datarate;
bool     SX1276_LowDatarateOptimize;
uint8_t  SX1276_Coderate;
uint16_t SX1276_PreambleLen;
bool     SX1276_FixLen;
uint8_t  SX1276_PayloadLen;
bool     SX1276_CrcOn;
bool     SX1276_FreqHopOn;
uint8_t  SX1276_HopPeriod;
bool     SX1276_IqInverted;
bool     SX1276_RxContinuous;
uint32_t SX1276_TxTimeout;
int8_t   SX1276_Payload_SnrValue;
int8_t   SX1276_Payload_RssiValue;
uint8_t  SX1276_Payload_Size;
uint8_t  SX1276_RxTx;








/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;



uint8_t work_frequence;





void SX1276SetOpMode( uint8_t opMode );


#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}       
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define NOISE_ABSOLUTE_ZERO                         -174.0
#define NOISE_FIGURE_LF                             4.0
#define NOISE_FIGURE_HF                             6.0
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0


#define RX_BUFFER_SIZE                              256
static uint8_t SX1276_RxBuf[RX_BUFFER_SIZE]; //Reception buffer






/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;
TimerEvent_t RxTimeoutSyncWord;

//Writes the radio register at the specified address
void SX1276Write( uint8_t addr, uint8_t data )
{
  SPI_Send( addr, &data, 1 );
}
//Reads the radio register at the specified address
uint8_t SX1276Read( uint8_t addr )
{
  uint8_t data;
  SPI_Read( addr, &data, 1 );
  return data;
}


void SX1276_Init( void )
{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);	 //PA2 = 1: power on, PA2 = 0: power off.
	  
    TimerInit( &TxTimeoutTimer, SX1276OnTimeoutIrq );// Initialize driver timeout timers
    TimerInit( &RxTimeoutTimer, SX1276OnTimeoutIrq );
    TimerInit( &RxTimeoutSyncWord, SX1276OnTimeoutIrq );

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);// Set RESET pin to 0
		DelayMs( 1 );// Wait 1 ms
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);// Configure RESET as input
		DelayMs( 6 );// Wait 6 ms

	
		//SX1276SetTcxoConfig(); //add by lhb 2015.11.27
		SX1276Write( REG_TCXO, 0x19 ); //设置外部有源晶振  
		//SX1276Write(REG_PACONFIG,0xff);//接到P A 口

  	//RxChainCalibration( );接收频率校准Performs the Rx chain calibration for LF and HF bands,\remark Must be called just after the reset so all registers are at their default values
    uint8_t regPaConfigInitVal;
    regPaConfigInitVal = SX1276Read( REG_PACONFIG );// Save context
    SX1276Write( REG_PACONFIG, 0x00 );// Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );// Launch Rx chain calibration for LF band
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );// Launch Rx chain calibration for HF band
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }
    SX1276Write( REG_PACONFIG, regPaConfigInitVal );// Restore context

    for(uint8_t i=0;i<sizeof(RadioRegsInit)/sizeof(RadioRegisters_t);i++)
    {
			SX1276SetModem( RadioRegsInit[i].Modem );
			SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }
    SX1276SetModem( MODEM_FSK );
    SX1276_State = RF_IDLE;
					
		srand( RAND_SEED );// Random seed initialization
		SX1276SetModem( MODEM_LORA );
		SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );// Change LoRa modem SyncWord
	 //SX1276Write( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );// Change LoRa modem SyncWord
	 //SX1276SetSleep( );   //20160317					
}




//Generates a 32 bits random value based on the RSSI readings\remark This function sets the radio in LoRa modem mode and disables  all interrupts. After calling this function either Radio.SetRxConfig or Radio.SetTxConfig functions must be called.									
uint32_t SX1276Random( void ) 
{
		uint32_t rnd = 0; 
		SX1276SetModem( MODEM_LORA );// Set LoRa modem ON
		SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |RFLR_IRQFLAGS_RXDONE |RFLR_IRQFLAGS_PAYLOADCRCERROR |RFLR_IRQFLAGS_VALIDHEADER |RFLR_IRQFLAGS_TXDONE |RFLR_IRQFLAGS_CADDONE |RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |RFLR_IRQFLAGS_CADDETECTED );// Disable LoRa modem interrupts
		SX1276SetOpMode( RF_OPMODE_RECEIVER );// Set radio in continuous reception
		for( uint8_t i = 0; i < 32; i++ )
		{
				DelayMs( 1 );  
				rnd |= ((uint32_t)SX1276Read(REG_LR_RSSIWIDEBAND)&0x01)<<i;// Unfiltered RSSI value reading. Only takes the LSB value
		}
		SX1276SetSleep( );	
		return rnd;
}	



#define RSSI_FREE_TH                ( int8_t )( -90 ) // [dBm]  //RSSI free threshold
bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq)
{
    int8_t rssi = 0; //eadi by hxz 20140825
   // int16_t rssi = 0; //add by hxz 20140825
    SX1276SetModem( modem );
	
    //SX1276SetChannel( freq );
    SX1276_Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
	
    SX1276SetOpMode( RF_OPMODE_RECEIVER );
    DelayMs( 1 );
    rssi = SX1276ReadRssi( modem );
    SX1276SetSleep( );
    if( rssi > RSSI_FREE_TH )
    {
        return false;
    }
    return true;
}




void SX1276_RxConfig( RadioModems_t modem, uint32_t freq,uint32_t bandwidth,uint32_t datarate, uint8_t coderate,uint32_t bandwidthAfc, uint16_t preambleLen,uint16_t symbTimeout, bool fixLen,uint8_t payloadLen,bool crcOn, bool freqHopOn, uint8_t hopPeriod,bool iqInverted, bool rxContinuous,uint32_t timeout )
{
	
   //SetChannel( uint32_t freq )	
    SX1276_Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );

	
    SX1276SetModem( modem );
		if( bandwidth > 2 )	// Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
		{
				while( 1 );
		}
		bandwidth += 7;
		SX1276_Bandwidth = bandwidth;
		SX1276_Datarate = datarate;
		SX1276_Coderate = coderate;
		SX1276_FixLen = fixLen;
		SX1276_PayloadLen = payloadLen;
		SX1276_CrcOn  = crcOn;
		SX1276_FreqHopOn = freqHopOn;
		SX1276_HopPeriod = hopPeriod;
		SX1276_IqInverted = iqInverted;
		SX1276_RxContinuous = rxContinuous;


		if( datarate > 12 )
				datarate = 12;
		else if( datarate < 6 )
				datarate = 6;

		if(((bandwidth==7)&&((datarate==11)||(datarate==12)))||((bandwidth==8)&&(datarate ==12)))
				SX1276_LowDatarateOptimize = 0x01;
		else
				SX1276_LowDatarateOptimize = 0x00;

		SX1276Write( REG_LR_MODEMCONFIG1,( SX1276Read( REG_LR_MODEMCONFIG1 ) &RFLR_MODEMCONFIG1_BW_MASK &RFLR_MODEMCONFIG1_CODINGRATE_MASK &RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |( bandwidth << 4 ) | ( coderate << 1 ) |fixLen );
		SX1276Write( REG_LR_MODEMCONFIG2,( SX1276Read( REG_LR_MODEMCONFIG2 ) &RFLR_MODEMCONFIG2_SF_MASK &RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |( datarate << 4 ) | ( crcOn << 2 ) |( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );
		SX1276Write( REG_LR_MODEMCONFIG3,( SX1276Read( REG_LR_MODEMCONFIG3 ) &RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |( SX1276_LowDatarateOptimize << 3 ) );
		SX1276Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );
		SX1276Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
		SX1276Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

		if( fixLen == 1 )
				SX1276Write( REG_LR_PAYLOADLENGTH, payloadLen );

		if( SX1276_FreqHopOn == true )
		{
				SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
				SX1276Write( REG_LR_HOPPERIOD, SX1276_HopPeriod );
		}

		if( ( bandwidth == 9 ) && (RF_MID_BAND_THRESH ) )
		{
				// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
				SX1276Write( REG_LR_TEST36, 0x02 );
				SX1276Write( REG_LR_TEST3A, 0x64 );
		}
		else if( bandwidth == 9 )
		{
				// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
				SX1276Write( REG_LR_TEST36, 0x02 );
				SX1276Write( REG_LR_TEST3A, 0x7F );
		}
		else
		{
				// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
				SX1276Write( REG_LR_TEST36, 0x03 );
		}

		if( datarate == 6 )
		{
				SX1276Write( REG_LR_DETECTOPTIMIZE, ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &RFLR_DETECTIONOPTIMIZE_MASK ) |RFLR_DETECTIONOPTIMIZE_SF6 );
				SX1276Write( REG_LR_DETECTIONTHRESHOLD,RFLR_DETECTIONTHRESH_SF6 );
		}
		else
		{
				SX1276Write( REG_LR_DETECTOPTIMIZE,( SX1276Read( REG_LR_DETECTOPTIMIZE ) &RFLR_DETECTIONOPTIMIZE_MASK ) |RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
				SX1276Write( REG_LR_DETECTIONTHRESHOLD,RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
		}


		
//SX1276SetRx( uint32_t timeout )
		
    bool SetRx_rxContinuous = false;

		if( SX1276_IqInverted == true )
		{
				SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
				SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
		}
		else
		{
				SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
				SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
		}

		SetRx_rxContinuous = SX1276_RxContinuous;

		if( SX1276_FreqHopOn == true )
		{
				SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
																					//RFLR_IRQFLAGS_RXDONE |
																					//RFLR_IRQFLAGS_PAYLOADCRCERROR |
																					RFLR_IRQFLAGS_VALIDHEADER |
																					RFLR_IRQFLAGS_TXDONE |
																					RFLR_IRQFLAGS_CADDONE |
																					//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
																					RFLR_IRQFLAGS_CADDETECTED );

				// DIO0=RxDone, DIO2=FhssChangeChannel
				SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
		}
		else
		{
				SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
																					//RFLR_IRQFLAGS_RXDONE |
																					//RFLR_IRQFLAGS_PAYLOADCRCERROR |
																					RFLR_IRQFLAGS_VALIDHEADER |
																					RFLR_IRQFLAGS_TXDONE |
																					RFLR_IRQFLAGS_CADDONE |
																					RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
																					RFLR_IRQFLAGS_CADDETECTED );

				// DIO0=RxDone
				SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
		}
		SX1276Write( REG_LR_FIFORXBASEADDR, 0 );
		SX1276Write( REG_LR_FIFOADDRPTR, 0 );

    mem_Init( SX1276_RxBuf, 0, ( size_t )RX_BUFFER_SIZE );

    SX1276_State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerEvent_Start( &RxTimeoutTimer );
    }

		if( SetRx_rxContinuous == true )
		{
				SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
		}
		else
		{
				SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
		}	
}


void SX1276_TxConfig( RadioModems_t modem, uint32_t freq,int8_t power, uint32_t fdev,uint32_t bandwidth, uint32_t datarate,uint8_t coderate, uint16_t preambleLen,bool fixLen, bool crcOn, bool freqHopOn,uint8_t hopPeriod, bool iqInverted, uint32_t timeout,uint64_t *GetTimeOnAir,uint8_t pktLen )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

	
   //SetChannel( uint32_t freq )	
    SX1276_Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );	
	
	
    SX1276SetModem( modem );
	
		//SX1276SetRfTxPower
    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );
    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        else
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
                power = 5;
            if( power > 20 )
                power = 20;
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
                power = 2;
            if( power > 17 )
                power = 17;
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
            power = -1;
        if( power > 14 )
            power = 14;
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
		SX1276_Power = power;
		
		
		if( bandwidth > 2 )
				while( 1 );// Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
		bandwidth += 7;
		SX1276_Bandwidth = bandwidth;
		SX1276_Datarate = datarate;
		SX1276_Coderate = coderate;
		SX1276_PreambleLen = preambleLen;
		SX1276_FixLen = fixLen;
		SX1276_FreqHopOn = freqHopOn;
		SX1276_HopPeriod = hopPeriod;
		SX1276_CrcOn  = crcOn;
		SX1276_IqInverted = iqInverted;
		//SX1276_TxTimeout = timeout;
		SX1276_TxTimeout = 0x007A1200 ;


		if( datarate > 12 )
				datarate = 12;
		else if( datarate < 6 )
				datarate = 6;
		if((( bandwidth==7)&&((datarate==11)||(datarate ==12)))||((bandwidth==8)&&(datarate==12)))
				SX1276_LowDatarateOptimize = 0x01;
		else
				SX1276_LowDatarateOptimize = 0x00;


		if( SX1276_FreqHopOn == true )
		{
				SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
				SX1276Write( REG_LR_HOPPERIOD, SX1276_HopPeriod );
		}

		SX1276Write( REG_LR_MODEMCONFIG1,( SX1276Read( REG_LR_MODEMCONFIG1 ) &RFLR_MODEMCONFIG1_BW_MASK &RFLR_MODEMCONFIG1_CODINGRATE_MASK &RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |( bandwidth << 4 ) | ( coderate << 1 ) |fixLen );
		SX1276Write( REG_LR_MODEMCONFIG2,( SX1276Read( REG_LR_MODEMCONFIG2 ) &RFLR_MODEMCONFIG2_SF_MASK &RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |( datarate << 4 ) | ( crcOn << 2 ) );
		SX1276Write( REG_LR_MODEMCONFIG3,( SX1276Read( REG_LR_MODEMCONFIG3 ) &RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |( SX1276_LowDatarateOptimize << 3 ) );
		SX1276Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
		SX1276Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

		if( datarate == 6 )
		{
				SX1276Write( REG_LR_DETECTOPTIMIZE,( SX1276Read( REG_LR_DETECTOPTIMIZE ) &RFLR_DETECTIONOPTIMIZE_MASK ) |RFLR_DETECTIONOPTIMIZE_SF6 );
				SX1276Write( REG_LR_DETECTIONTHRESHOLD,RFLR_DETECTIONTHRESH_SF6 );
		}
		else
		{
				SX1276Write( REG_LR_DETECTOPTIMIZE,( SX1276Read( REG_LR_DETECTOPTIMIZE ) &RFLR_DETECTIONOPTIMIZE_MASK ) |RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
				SX1276Write( REG_LR_DETECTIONTHRESHOLD,RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
		}
		

    //SX1276GetTimeOnAir
		double bw = 0.0;
		// REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
		switch( SX1276_Bandwidth )
		{
		//case 0: // 7.8 kHz
		//    bw = 78e2;
		//    break;
		//case 1: // 10.4 kHz
		//    bw = 104e2;
		//    break;
		//case 2: // 15.6 kHz
		//    bw = 156e2;
		//    break;
		//case 3: // 20.8 kHz
		//    bw = 208e2;
		//    break;
		//case 4: // 31.2 kHz
		//    bw = 312e2;
		//    break;
		//case 5: // 41.4 kHz
		//    bw = 414e2;
		//    break;
		//case 6: // 62.5 kHz
		//    bw = 625e2;
		//    break;
		case 7: // 125 kHz
				bw = 125e3;
				break;
		case 8: // 250 kHz
				bw = 250e3;
				break;
		case 9: // 500 kHz
				bw = 500e3;
				break;
		}

		// Symbol rate : time for one symbol (secs)
		double rs = bw / ( 1 << SX1276_Datarate );
		double ts = 1 / rs;
		// time of preamble
		double tPreamble = ( SX1276_PreambleLen + 4.25 ) * ts;
		// Symbol length of payload and time
		double tmp = ceil( ( 8 * pktLen - 4 * SX1276_Datarate +
												 28 + 16 * SX1276_CrcOn  -
												 ( SX1276_FixLen ? 20 : 0 ) ) /
												 ( double )( 4 * SX1276_Datarate -
												 ( ( SX1276_LowDatarateOptimize > 0 ) ? 8 : 0 ) ) ) *
												 ( SX1276_Coderate + 4 );
		double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
		double tPayload = nPayload * ts;
		double tOnAir = tPreamble + tPayload;// Time on air	
		*GetTimeOnAir = floor( tOnAir * 1e6 + 0.999 );// return us secs
}


bool    (*CheckRfFrequency )( uint32_t frequency );
    


void SX1276SetSleep( void )
{
    TimerEvent_Stop( &RxTimeoutTimer );
    TimerEvent_Stop( &TxTimeoutTimer );

    SX1276SetOpMode( RF_OPMODE_SLEEP );
    SX1276_State = RF_IDLE;
}

void SX1276SetStby( void )
{
    TimerEvent_Stop( &RxTimeoutTimer );
    TimerEvent_Stop( &TxTimeoutTimer );

    SX1276SetOpMode( RF_OPMODE_STANDBY );
    SX1276_State = RF_IDLE;
}



void SX1276StartCad( void )
{
		SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
																RFLR_IRQFLAGS_RXDONE |
																RFLR_IRQFLAGS_PAYLOADCRCERROR |
																RFLR_IRQFLAGS_VALIDHEADER |
																RFLR_IRQFLAGS_TXDONE |
																//RFLR_IRQFLAGS_CADDONE |
																RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
																//RFLR_IRQFLAGS_CADDETECTED
																);

		// DIO3=CADDone
		SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );

		SX1276_State = RF_CAD;
		SX1276SetOpMode( RFLR_OPMODE_CAD );
}

int8_t SX1276ReadRssi( RadioModems_t modem )
{
    int8_t rssi = 0; 
		if( SX1276_Channel > RF_MID_BAND_THRESH )
		{
				rssi = RSSI_OFFSET_HF + SX1276Read( REG_LR_RSSIVALUE );
		}
		else
		{
				rssi = RSSI_OFFSET_LF + SX1276Read( REG_LR_RSSIVALUE );
		}
    return rssi;
}



void SX1276SetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RF_OPMODE_STANDBY;//待机模式
    if( opMode != opModePrev )
    {
        opModePrev = opMode;
        if( opMode == RF_OPMODE_SLEEP )
        {
            //SX1276SetAntSwLowPower( true );void SX1276SetAntSwLowPower( bool status )
						if( RadioIsActive != true )
						{
								RadioIsActive = true;
								//SX1276AntSwDeInit( );
								//RADIO_ANT_SWITCH_HF -- PA1
								GPIO_InitTypeDef  GPIO_InitStruct;
								//__GPIOA_CLK_DISABLE();
								GPIO_InitStruct.Pin = (GPIO_PIN_1);
								GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
								GPIO_InitStruct.Pull = GPIO_NOPULL;
								GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
								HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //add by zjh	
            }
				}
        else
        {
            //SX1276SetAntSwLowPower( false );void SX1276SetAntSwLowPower( bool status )	
						if( RadioIsActive != false )
						{
								RadioIsActive = false;
								//SX1276AntSwInit( );
								//RADIO_ANT_SWITCH_HF -- PA1
								 GPIO_InitTypeDef  GPIO_InitStruct;
								__GPIOA_CLK_ENABLE();
								GPIO_InitStruct.Pin = (GPIO_PIN_1);
								GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
								GPIO_InitStruct.Pull = GPIO_PULLUP;
								GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
								HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //add by zjh	
						}
							 
            if( opMode == RF_OPMODE_TRANSMITTER ) //TX MODE
            {
								if( SX1276_RxTx != 1 )
								{
									SX1276_RxTx = 1;
									HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
								}
            }
            else
            {
								if( SX1276_RxTx != 0 )
								{
									SX1276_RxTx = 0;
									HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);										
								}
            }
        }
        SX1276Write(REG_OPMODE,(SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
    }
}
		


    /*!
     * \brief Configures the radio with the given modem
     *
     * \param [IN] modem Modem to be used [0: FSK, 1: LoRa] 
     */
void SX1276SetModem( RadioModems_t modem )
{
    if( SX1276_Modem == modem )
    {
        return;
    }

    SX1276_Modem = modem;
    switch( SX1276_Modem )
    {
    default:
    case MODEM_FSK:
        SX1276SetOpMode( RF_OPMODE_SLEEP );  //20160317
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1276SetOpMode( RF_OPMODE_SLEEP );   //20160317
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}



void SX1276OnTimeoutIrq( void )
{
    switch( SX1276_State )
    {
    case RF_RX_RUNNING:
        OnRadioRxTimeout( );
        break;
    case RF_TX_RUNNING:
        SX1276_State = RF_IDLE;
        OnRadioTxTimeout( );
        break;
    default:
        break;
    }
}

void SX1276OnDio0Irq( void )
{
	   int16_t rssi;
    __IO uint8_t irqFlags = 0;
    int8_t snr = 0;
    switch( SX1276_State )
    {
        case RF_RX_RUNNING:// RxDone interrupt
            //TimerEvent_Stop( &RxTimeoutTimer );	
						SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );// Clear Irq
						irqFlags = SX1276Read( REG_LR_IRQFLAGS );
						if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
						{				
								SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );// Clear Irq
								if( SX1276_RxContinuous == false )
								{
										SX1276_State = RF_IDLE;
								}
								TimerEvent_Stop( &RxTimeoutTimer );
								OnRadioRxError( );
								break;
						}

						SX1276_Payload_SnrValue = SX1276Read( REG_LR_PKTSNRVALUE );
						if( SX1276_Payload_SnrValue & 0x80 ) // The SNR sign bit is 1
						{
								// Invert and divide by 4
								snr = ( ( ~SX1276_Payload_SnrValue + 1 ) & 0xFF ) >> 2;
								snr = -snr;
						}
						else
						{
								// Divide by 4
								snr = ( SX1276_Payload_SnrValue & 0xFF ) >> 2;
						}

						rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
						if( snr < 0 )
						{
								if( SX1276_Channel > RF_MID_BAND_THRESH )
								{
										SX1276_Payload_RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
																																	snr;
								}
								else
								{
										SX1276_Payload_RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
																																	snr;
								}
						}
						else
						{
								if( SX1276_Channel > RF_MID_BAND_THRESH )
								{
										SX1276_Payload_RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
								}
								else
								{
										SX1276_Payload_RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
								}
						}

						SX1276_Payload_Size = SX1276Read( REG_LR_RXNBBYTES );
						SPI_Read( 0, SX1276_RxBuf, SX1276_Payload_Size );   //SX1276ReadFifo( SX1276_RxBuf, SX1276_Payload_Size );
						

						if( SX1276_RxContinuous == false )
						{
								SX1276_State = RF_IDLE;
						}
						TimerEvent_Stop( &RxTimeoutTimer );
						OnRadioRxDone( SX1276_RxBuf, SX1276_Payload_Size, SX1276_Payload_RssiValue, SX1276_Payload_SnrValue );            

				
				
            break;
        case RF_TX_RUNNING:// TxDone interrupt
            TimerEvent_Stop( &TxTimeoutTimer );
            switch( SX1276_Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                SX1276_State = RF_IDLE;
                OnRadioTxDone( );
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio1Irq( void )
{
    switch( SX1276_State )
    {
        case RF_RX_RUNNING:
						// Sync time out
						TimerEvent_Stop( &RxTimeoutTimer );
						SX1276_State = RF_IDLE;
						OnRadioRxTimeout( );
            break;
        case RF_TX_RUNNING:
            break;
        default:
            break;
    }
}


void SX1276OnDio2Irq( void )
{
    switch( SX1276_State )
    {
        case RF_RX_RUNNING:
						if( SX1276_FreqHopOn == true )
						{
								// Clear Irq
								SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
								OnRadioFhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
						}
            break;
        case RF_TX_RUNNING:
						if( SX1276_FreqHopOn == true )
						{
								// Clear Irq
								SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
								OnRadioFhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
						}
            break;
        default:
            break;
    }
}

void SX1276OnDio3Irq( void )
{
	if( ( SX1276Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
	{
			// Clear Irq
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
			OnRadioCadDone( true );
	}
	else
	{
			// Clear Irq
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
			OnRadioCadDone( false );
	}
}

void SX1276OnDio4Irq( void )
{
}

void SX1276OnDio5Irq( void )
{
}

void SX1276_Send(uint8_t *buffer, uint8_t size)
{
		uint32_t txTimeout = 0;
		if( SX1276_IqInverted == true )
		{
				SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
				SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
		}
		else
		{
				SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
				SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
		}
		SX1276_Payload_Size = size;
		SX1276Write( REG_LR_PAYLOADLENGTH, size );// Initializes the payload size
		SX1276Write( REG_LR_FIFOTXBASEADDR, 0 );// Full buffer used for Tx
		SX1276Write( REG_LR_FIFOADDRPTR, 0 );
		if( ( SX1276Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
		{
				SX1276SetStby( );  //FIFO operations can not take place in Sleep mode
				DelayMs( 1 );
		}	
		SPI_Send(0,buffer, size );// Write payload buffer
		txTimeout = SX1276_TxTimeout;
		
		//sx1276 SetTx

		if( SX1276_FreqHopOn == true )
		{
				SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |RFLR_IRQFLAGS_RXDONE |RFLR_IRQFLAGS_PAYLOADCRCERROR |RFLR_IRQFLAGS_VALIDHEADER |RFLR_IRQFLAGS_CADDONE |RFLR_IRQFLAGS_CADDETECTED );//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |RFLR_IRQFLAGS_TXDONE
				SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );// DIO0=TxDone, DIO2=FhssChangeChannel
		}
		else
		{
				SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |RFLR_IRQFLAGS_RXDONE |RFLR_IRQFLAGS_PAYLOADCRCERROR |RFLR_IRQFLAGS_VALIDHEADER |RFLR_IRQFLAGS_CADDONE |RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |RFLR_IRQFLAGS_CADDETECTED );//RFLR_IRQFLAGS_TXDONE |	
				SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );// DIO0=TxDone
		}
		SX1276_State = RF_TX_RUNNING;
		TimerSetValue( &TxTimeoutTimer, txTimeout );		
		TimerEvent_Start(&TxTimeoutTimer); //启动TxTimeoutTimer，清零(TxDone)
		SX1276SetOpMode(RF_OPMODE_TRANSMITTER ); 	
}


//Low Power Set
void LoRaMac_setlowPowerMode(u8 enable)
{
	if(enable == 1)
	{
		SX1276SetSleep(); 
		HAL_Delay(100);//????需要么?
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	}
	else
	{ /*
		//PA2控制sx芯片的外部晶振电源
		__GPIOA_CLK_ENABLE();
		GPIO_InitStruct.Pin = (GPIO_PIN_2);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/

		SX1276Write(REG_LR_TCXO,0x19);//配置为有源时钟
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
		//打开有源晶振电源
		//DelayMs( 5 );
	}
}



