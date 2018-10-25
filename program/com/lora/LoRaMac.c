#include "sx1276.h"
#include "../com.h"

//define TxPower ，发送功率
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };
#define TX_POWER_MAX_INDEX					5
#define TX_POWER_20_DBM             0
#define TX_POWER_14_DBM             1
#define TX_POWER_11_DBM             2
#define TX_POWER_08_DBM            	3
#define TX_POWER_05_DBM             4
#define TX_POWER_02_DBM             5
#define LORAMAC_MIN_TX_POWER  TX_POWER_02_DBM //Minimal Tx output power that can be used by the node
#define LORAMAC_MAX_TX_POWER  TX_POWER_20_DBM  //Minimal Tx output power that can be used by the node
#define LORAMAC_DEFAULT_TX_POWER   TX_POWER_14_DBM  //Default Tx output power used by the node
static int8_t ChannelsTxPower = LORAMAC_DEFAULT_TX_POWER; //Channels Tx output power

const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };
#define DR_0  0  // SF12 - BW125
#define DR_1  1  // SF11 - BW125
#define DR_2  2  // SF10 - BW125
#define DR_3  3  // SF9  - BW125
#define DR_4  4  // SF8  - BW125
#define DR_5  5  // SF7  - BW125
#define DR_6  6  // SF7  - BW250
#define DR_7  7  // FSK
#define LORAMAC_MIN_DATARATE  DR_0  //Minimal datarate that can be used by the node
#define LORAMAC_MAX_DATARATE  DR_7  //Minimal datarate that can be used by the node
#define LORAMAC_DEFAULT_DATARATE    DR_0//DR_5 Default datarate used by the nodes
static int8_t ChannelsDatarate = LORAMAC_DEFAULT_DATARATE; //Channels datarate
static int8_t ChannelsDefaultDatarate = LORAMAC_DEFAULT_DATARATE; //Channels defualt datarate


#define LC( channelIndex )            ( uint16_t )( 1 << ( channelIndex - 1 ) )

#define LORAMAC_PHY_MAXPAYLOAD                      128//64


#define RSSI_FREE_TH                ( int8_t )( -90 ) // [dBm]  //RSSI free threshold
#define UP_LINK                     0
#define DOWN_LINK                   1
#define LORAMAC_MFR_LEN             4//Sets the length of the LoRaMAC footer field. Mainly indicates the MIC field length



typedef union
{
    int8_t Value;
    struct
    {
        int8_t Min : 4;
        int8_t Max : 4;
    }PACKED Fields;
}PACKED DrRange_t;
typedef struct
{
    uint32_t Frequency; // Hz
    DrRange_t DrRange;  // Max datarate [0: SF12, 1: SF11, 2: SF10, 3: SF9, 4: SF8, 5: SF7, 6: SF7, 7: FSK]
                        // Min datarate [0: SF12, 1: SF11, 2: SF10, 3: SF9, 4: SF8, 5: SF7, 6: SF7, 7: FSK]
    uint8_t Band;       // Band index
}PACKED ChannelParams_t;



/*
//ZTE-0TAA
#define OVER_THE_AIR_ACTIVATION  1
uint32_t LoRaMacDevAddr = 0x28AD9151 ;
uint8_t LoRaMacNwkSKey[] ={0x3a, 0x1e, 0xbd, 0x00, 0x3a, 0xc8, 0x8c, 0x00, 0xc0, 0xc5, 0x1e, 0xff, 0xea, 0xd2, 0x65, 0x51  };//kiwi
uint8_t LoRaMacAppSKey[] ={0xe0, 0x8c, 0x28, 0xff, 0xcf, 0x7a, 0x47, 0xff, 0x7a, 0xb5, 0xeb, 0x00, 0xbd, 0x00, 0x01, 0x51  };//KIWI
uint8_t LoRaMacDevEui[] ={0x01,0x00,0x11,0xab,0x00,0x77,0x4a,0x00 };//ZTE
uint8_t LoRaMacAppEui[] ={0x11, 0x00, 0x00, 0xab, 0x00, 0xc5, 0x26, 0x2c  };//ZTE
uint8_t LoRaMacAppKey[]={0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };//for ZTE
uint32_t LoRaMacNetID = 0xAA5E1A;
#define LORA_MAX_NB_CHANNELS  16 
ChannelParams_t Channels[LORA_MAX_NB_CHANNELS]=
{
{ 482300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 482500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 482700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 482900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 483100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 483300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 483500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 483700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
};
ChannelParams_t DnChannels[16]=
{
{ 502100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 502300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 502500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 502700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 502900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 503100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 503300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 503500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
};
*/


//KIWI-ABP
#define OVER_THE_AIR_ACTIVATION  0
uint32_t LoRaMacDevAddr = 0x28AD9151 ;
uint8_t LoRaMacNwkSKey[] ={0x3a, 0x1e, 0xbd, 0x00, 0x3a, 0xc8, 0x8c, 0x00, 0xc0, 0xc5, 0x1e, 0xff, 0xea, 0xd2, 0x65, 0x51  };//kiwi
uint8_t LoRaMacAppSKey[] ={0xe0, 0x8c, 0x28, 0xff, 0xcf, 0x7a, 0x47, 0xff, 0x7a, 0xb5, 0xeb, 0x00, 0xbd, 0x00, 0x01, 0x51  };//KIWI
uint8_t LoRaMacDevEui[] ={0x70, 0xb3, 0xd5, 0x31, 0xc0, 0x00, 0x01, 0x51 };  
uint8_t LoRaMacAppEui[] ={0x70,0xb3,0xd5,0x31,0xc0,0x00,0x00,0x01};          
uint8_t LoRaMacAppKey[]={0xb3,0x19,0x0f,0xff,0x13,0xd6,0x93,0x00,0xad,0xa1,0x09,0xff,0x24,0x3e,0xa5,0x01 };
uint32_t LoRaMacNetID = 0xAA5E1A;
#define LORA_MAX_NB_CHANNELS  3 
ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
{ 470300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 470500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 470700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }, 
};
ChannelParams_t DnChannels[LORA_MAX_NB_CHANNELS]=
{
{ 500300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 500500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 500700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },      
};



/*
//KIWI-OTAA
#define OVER_THE_AIR_ACTIVATION  1
uint32_t LoRaMacDevAddr = 0x28AD9150 ;
uint8_t LoRaMacNwkSKey[] ={0x3a, 0x1e, 0xbd, 0x00, 0x3a, 0xc8, 0x8c, 0x00, 0xc0, 0xc5, 0x1e, 0xff, 0xea, 0xd2, 0x65, 0x50  };//kiwi
uint8_t LoRaMacAppSKey[] ={0xe0, 0x8c, 0x28, 0xff, 0xcf, 0x7a, 0x47, 0xff, 0x7a, 0xb5, 0xeb, 0x00, 0xbd, 0x00, 0x01, 0x50  };//KIWI
uint8_t LoRaMacDevEui[] ={0x70, 0xb3, 0xd5, 0x31, 0xc0, 0x00, 0x01, 0x50 };  
uint8_t LoRaMacAppEui[] ={0x70,0xb3,0xd5,0x31,0xc0,0x00,0x00,0x01};          
uint8_t LoRaMacAppKey[]={0xb3,0x19,0x0f,0xff,0x13,0xd6,0x93,0x00,0xad,0xa1,0x09,0xff,0x24,0x3e,0xa5,0x01 };
uint32_t LoRaMacNetID = 0xAA5E1A;
#define LORA_MAX_NB_CHANNELS  3 
ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
{ 470300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 470500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 470700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }, 
};
ChannelParams_t DnChannels[LORA_MAX_NB_CHANNELS]=
{
{ 500300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 500500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 500700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },      
};
*/




typedef struct
{
    uint32_t Frequency; // Hz
    uint8_t  Datarate;  // [0: SF12, 1: SF11, 2: SF10, 3: SF9, 4: SF8, 5: SF7, 6: SF7, 7: FSK]
}PACKED Rx2ChannelParams_t;
static Rx2ChannelParams_t Rx2Channel = { 471725000, DR_0 }; //LoRaMAC 2nd reception window settings


typedef struct
{
    uint16_t DCycle;
    int8_t TxMaxPower;
    uint64_t LastTxDoneTime;
    uint64_t TimeOff;
}PACKED Band_t;
#define LORA_MAX_NB_BANDS 					1//only one band is supported currentlly.
Band_t Bands[LORA_MAX_NB_BANDS] =
{
		{ 1, TX_POWER_14_DBM, 0,  0 },
};


typedef struct MulticastParams_s
{
    uint32_t Address;
    uint8_t NwkSKey[16];
    uint8_t AppSKey[16];
    uint32_t DownLinkCounter;
    struct MulticastParams_s *Next;
}PACKED MulticastParams_t;
static MulticastParams_t *MulticastChannels = NULL;

              

uint64_t TxTimeOnAir = 0;        //Last transmission time on air
static uint16_t LoRaMacDevNonce;

static uint8_t LoRaMacBuffer[LORAMAC_PHY_MAXPAYLOAD];
static uint16_t LoRaMacBufferPktLen = 0;
static uint8_t LoRaMacPayload[LORAMAC_PHY_MAXPAYLOAD];
static uint8_t LoRaMacRxPayload[LORAMAC_PHY_MAXPAYLOAD];

static uint32_t DownLinkCounter = 0;
static bool IsUpLinkCounterFixed = false;
static bool AdrCtrlOn = false;
static bool NodeAckRequested = false;
static bool SrvAckRequested = false;
static uint32_t AdrAckCounter = 0;



static uint8_t Rx1DrOffset = 0; //Datarate offset between uplink and downlink on first window
static uint16_t ChannelsMask;   //Mask indicating which channels are enabled
static uint8_t ChannelsNbRep = 1; //Number of uplink messages repetitions [1:15] (unconfirmed messages only)
static uint8_t ChannelsNbRepCounter = 0; //Uplink messages repetitions counter重复接收
static uint8_t Channel;  //Current channel index
static TimerEvent_t ChannelCheckTimer;  //LoRaMac channel check timer
static void OnChannelCheckTimerEvent( void ); //Function executed on channel check timer event

//入网
bool IsLoRaMacNetworkJoined = false;
TimerEvent_t JoinReqTimer;
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE          10000000  // 10 [s] value in us
void LoRaMacJoinReq(void);

//发送
void LoRaMac_Send(uint8_t *fBuffer, uint16_t fBufferSize, bool confirmed);
uint8_t LoRaMacSendFrame( uint8_t fPort, void *fBuffer, uint16_t fBufferSize );
uint8_t LoRaMacSendFrameOnChannel( ChannelParams_t channel );
uint32_t UpLinkCounter = 0;

//延迟发送
static bool DutyCycleOn; //Enables/Disables duty cycle management (Test only)
static uint8_t MaxDCycle = 0;   //Maximum duty cycle，\remark Possibility to shutdown the device. Everything else not implemented.
static uint16_t AggregatedDCycle;    //Agregated duty cycle management
static uint64_t AggregatedLastTxDoneTime;
static uint64_t AggregatedTimeOff;
static TimerEvent_t TxDelayedTimer;     //LoRaMac duty cycle delayed Tx timer
static void OnTxDelayedTimerEvent( void ); //Function executed on duty cycle delayed Tx  timer event

//应答、重发
bool AckReceived;
static uint8_t AckTimeoutRetries = 3;        //重发最大次数
static uint8_t AckTimeoutRetriesCounter = 1; //重发次数
static bool AckTimeoutRetry = false;         //Indicates if the AckTimeout timer has expired or not
static TimerEvent_t AckTimeoutTimer;         //Acknowledge timeout timer. Used for packet retransmissions.
#define ACK_TIMEOUT                 2000000
#define ACK_TIMEOUT_RND             1000000
static void OnAckTimeoutTimerEvent( void ); //Function executed on AckTimeout timer event

//接收窗口1
#define MAX_RX_WINDOW               3000000   //Class A&B maximum receive window delay in ms，LoRaMac maximum time a reception window stays open
static TimerEvent_t RxWindowTimer1;     //LoRaMac reception windows timers
uint32_t JoinAcceptDelay1 = 5000000;
uint32_t ReceiveDelay1 = 1000000;   //单位：us
static uint32_t RxWindow1Delay;
static void OnRxWindow1TimerEvent( void ); //Function executed on first Rx window timer event

//接收窗口2
static TimerEvent_t RxWindowTimer2;
uint32_t JoinAcceptDelay2 = 6000000;
uint32_t ReceiveDelay2 = 2000000;
static uint32_t RxWindow2Delay;
static void OnRxWindow2TimerEvent( void ); //Function executed on second Rx window timer event




LoRaMacEventFlags_t LoRaMacEventFlags;  //LoRaMac notification event flags
//LoRaMAC events structure，Used to notify upper layers of MAC events

enum LoRaMacState_e   //LoRaMac internal states
{
        MAC_IDLE          = 0x00000000,
        MAC_TX_RUNNING    = 0x00000001,
        MAC_RX            = 0x00000002,
        MAC_ACK_REQ       = 0x00000004,
        MAC_ACK_RETRY     = 0x00000008,
        MAC_CHANNEL_CHECK = 0x00000010,
};



typedef struct
{
    uint8_t TxNbRetries;
    uint8_t TxDatarate;
    uint8_t RxPort;
    uint8_t *RxBuffer;
    uint8_t RxBufferSize;
    int16_t RxRssi;
    uint8_t RxSnr;
    uint16_t Energy;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}PACKED LoRaMacEventInfo_t;
LoRaMacEventInfo_t LoRaMacEventInfo;    //LoRaMac notification event info
uint32_t LoRaMacState = MAC_IDLE;       //LoRaMac internal state
static TimerEvent_t MacStateCheckTimer; //LoRaMac timer used to check the LoRaMacState (runs every second)
#define MAC_STATE_CHECK_TIMEOUT     700000 //Check the Mac layer state every MAC_STATE_CHECK_TIMEOUT
static void OnMacStateCheckTimerEvent( void );  //Function executed on Resend Frame timer event.








/*!
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define LORAMAC_MIC_BLOCK_B0_SIZE                   16
static uint8_t MicBlockB0[] = { 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t Mic[16]; //Contains the computed MIC field.\remark Only the 4 first bytes are used
static uint8_t aBlock[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t sBlock[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static aes_context AesContext;  //AES computation context variable
static AES_CMAC_CTX AesCmacCtx[1]; //CMAC computation context variable




/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           4150 // mV
#define BATTERY_MIN_LEVEL                           3200 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3100 // mV



typedef enum
{
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    SRV_MAC_LINK_ADR_REQ             = 0x03,
    SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
    SRV_MAC_RX_PARAM_SETUP_REQ       = 0x05,
    SRV_MAC_DEV_STATUS_REQ           = 0x06,
    SRV_MAC_NEW_CHANNEL_REQ          = 0x07,
    SRV_MAC_RX_TIMING_SETUP_REQ      = 0x08,
	 //SRV_MAC_GET_CHANNAL_INFO_ACK     = 0x8A,
	//SRV_MAC_UPDATA_CHANNAL_INFO_ACK  = 0x8B,
	//SRV_MAC_SYNC_TIME_INFO_ACK       = Ox8C,
	//SRV_MAC_GET_TIME_INFO_ACK        = 0x8D,
	//SRV_MAC_JUMBO_FRAME_ACK          = 0x8E,
}PACKED LoRaMacSrvCmd_t;
typedef enum
{
    MOTE_MAC_LINK_CHECK_REQ          = 0x02,
    MOTE_MAC_LINK_ADR_ANS            = 0x03,
    MOTE_MAC_DUTY_CYCLE_ANS          = 0x04,
    MOTE_MAC_RX_PARAM_SETUP_ANS      = 0x05,
    MOTE_MAC_DEV_STATUS_ANS          = 0x06,
    MOTE_MAC_NEW_CHANNEL_ANS         = 0x07,
    MOTE_MAC_RX_TIMING_SETUP_ANS     = 0x08,
}PACKED LoRaMacMoteCmd_t;
static bool MacCommandsInNextTx = false;
static uint8_t MacCommandsBufferIndex = 0;
static uint8_t MacCommandsBuffer[15];


uint16_t BoardGetPowerSupply( void )
{
    return 0 ;
}

uint8_t BoardMeasureBatterieLevel( void )
{
    __IO uint8_t batteryLevel = 0;
    uint16_t batteryVoltage = 0;

    if(1) //if( GpioRead( &UsbDetect ) == 1 )
    {
        batteryLevel = 0;
    }
    else
    {
        batteryVoltage = BoardGetPowerSupply( );

        if( batteryVoltage >= BATTERY_MAX_LEVEL )
        {
            batteryLevel = 254;
        }
        else if( ( batteryVoltage > BATTERY_MIN_LEVEL ) && ( batteryVoltage < BATTERY_MAX_LEVEL ) )
        {
            batteryLevel = ( ( 253 * ( batteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
        }
        else if( batteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
        {
            batteryLevel = 255;
            //GpioInit( &DcDcEnable, DC_DC_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
            //GpioInit( &BoardPowerDown, BOARD_POWER_DOWN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        }
        else // BATTERY_MIN_LEVEL
        {
            batteryLevel = 1;
        }
    }
    return batteryLevel;
}


/*!
 * Searches and set the next random available channel
 *
 * \retval status  Function status [0: OK, 1: Unable to find a free channel]
 */
static uint8_t LoRaMacSetNextChannel( void )
{
        uint8_t i = 0;
        uint8_t j = 0;
        uint8_t channelNext = Channel;
        uint8_t nbEnabledChannels = 0;
        uint8_t enabledChannels[LORA_MAX_NB_CHANNELS];
        uint64_t curTime = TIM2_Time_Present();

        memset1( enabledChannels, 0, LORA_MAX_NB_CHANNELS );

        // Update Aggregated duty cycle
        if( AggregatedTimeOff < ( curTime - AggregatedLastTxDoneTime ) )
        {
                AggregatedTimeOff = 0;
        }

        // Update bands Time OFF
        uint64_t nextTxDelay = 0xFFFFFFFFFFFFFFFF;
        for( i = 0; i < LORA_MAX_NB_BANDS; i++ )
        {
                if( DutyCycleOn == true )
                {
                        if( Bands[i].TimeOff < ( curTime - Bands[i].LastTxDoneTime ) )
                        {
                                Bands[i].TimeOff = 0;
                        }
                        if( Bands[i].TimeOff != 0 )
                        {
                                nextTxDelay = MIN( Bands[i].TimeOff, nextTxDelay );
                        }
                }
                else
                {
                        nextTxDelay = 0;
                        Bands[i].TimeOff = 0;
                }
        }

        // Search how many channels are enabled
        for( i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
        {
                if( ( ( 1 << i ) & ChannelsMask ) != 0 )
                {
                        if( Channels[i].Frequency == 0 )
                        {
                                // Check if the channel is enabled
                                continue;
                        }
                        if( ( ( Channels[i].DrRange.Fields.Min <= ChannelsDatarate ) &&
                                        ( ChannelsDatarate <= Channels[i].DrRange.Fields.Max ) ) == false )
                        {
                                // Check if the current channel selection supports the given datarate
                                continue;
                        }
                        if( Bands[Channels[i].Band].TimeOff > 0 )
                        {
                                // Check if the band is available for transmission
                                continue;
                        }
                        if( AggregatedTimeOff > 0 )
                        {
                                // Check if there is time available for transmission
                                continue;
                        }
                        enabledChannels[nbEnabledChannels++] = i;
                }
        }
        if( nbEnabledChannels > 0 )
        {
                for( i = 0, j = randr( 0, nbEnabledChannels - 1 ); i < LORA_MAX_NB_CHANNELS; i++ )
                {
                        channelNext = enabledChannels[j];
                        j = ( j + 1 ) % nbEnabledChannels;

                        if( SX1276IsChannelFree( MODEM_LORA, Channels[channelNext].Frequency, RSSI_FREE_TH ) == true )
                        {
                                // Free channel found
                                Channel = channelNext;

                                LoRaMacState &= ~MAC_CHANNEL_CHECK;
                                TimerEvent_Stop( &ChannelCheckTimer );
                                return 0;
                        }
                }
        }
        // No free channel found.
        // Check again
        if( ( LoRaMacState & MAC_CHANNEL_CHECK ) == 0 )
        {
                TimerSetValue( &ChannelCheckTimer, nextTxDelay );
                TimerEvent_Start( &ChannelCheckTimer );
                LoRaMacState |= MAC_CHANNEL_CHECK;
        }
        return 1;
}

/*
 * TODO: Add documentation
 */
void OnChannelCheckTimerEvent( void )
{
        LoRaMacState &= ~MAC_CHANNEL_CHECK;
        if( LoRaMacSetNextChannel( ) == 0 )
        {
                if( ( LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
                {
                        LoRaMacSendFrameOnChannel( Channels[Channel] );
                }
        }
}

/*!
 * Adds a new MAC command to be sent.
 *
 * \Remark MAC layer internal function
 *
 * \param [in] cmd MAC command to be added
 *                 [MOTE_MAC_LINK_CHECK_REQ,
 *                  MOTE_MAC_LINK_ADR_ANS,
 *                  MOTE_MAC_DUTY_CYCLE_ANS,
 *                  MOTE_MAC_RX2_PARAM_SET_ANS,
 *                  MOTE_MAC_DEV_STATUS_ANS
 *                  MOTE_MAC_NEW_CHANNEL_ANS]
 * \param [in] p1  1st parameter ( optional depends on the command )
 * \param [in] p2  2nd parameter ( optional depends on the command )
 *
 * \retval status  Function status [0: OK, 1: Unknown command, 2: Buffer full]
 */
static uint8_t AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 )
{
        MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
        switch( cmd )
        {
        case MOTE_MAC_LINK_CHECK_REQ:
                // No payload for this command
                break;
        case MOTE_MAC_LINK_ADR_ANS:
                // Margin
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                break;
        case MOTE_MAC_DUTY_CYCLE_ANS:
                // No payload for this answer
                break;
        case MOTE_MAC_RX_PARAM_SETUP_ANS:
                // Status: Datarate ACK, Channel ACK
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                break;
        case MOTE_MAC_DEV_STATUS_ANS:
                // 1st byte Battery
                // 2nd byte Margin
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                MacCommandsBuffer[MacCommandsBufferIndex++] = p2;
                break;
        case MOTE_MAC_NEW_CHANNEL_ANS:
                // Status: Datarate range OK, Channel frequency OK
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                break;
        case MOTE_MAC_RX_TIMING_SETUP_ANS:
                // No payload for this answer
                break;
        default:
                return 1;
        }
        if( MacCommandsBufferIndex <= 15 )
        {
                MacCommandsInNextTx = true;
                return 0;
        }
        else
        {
                return 2;
        }
}

// TODO: Add Documentation





volatile static bool TxDone = false;
static u32 Rxpacket_count = 0 ;


void LoRaMacInit(void)
{
	  SX1276_Init();
		DownLinkCounter = 0;
		LoRaMacState = MAC_IDLE;

		ChannelsMask = LC( 1 ) + LC( 2 ) + LC( 3 );

		ChannelsTxPower = LORAMAC_DEFAULT_TX_POWER;
		ChannelsDefaultDatarate = ChannelsDatarate = LORAMAC_DEFAULT_DATARATE;
		ChannelsNbRep = 1;
		ChannelsNbRepCounter = 0;

		MaxDCycle = 0;
		AggregatedDCycle = 1;
		AggregatedLastTxDoneTime = 0;
		AggregatedTimeOff = 0;

		DutyCycleOn = true;



		TimerInit( &MacStateCheckTimer, OnMacStateCheckTimerEvent );
		TimerInit( &ChannelCheckTimer, OnChannelCheckTimerEvent );
		TimerInit( &TxDelayedTimer, OnTxDelayedTimerEvent );
		TimerInit( &RxWindowTimer1, OnRxWindow1TimerEvent );
		TimerInit( &RxWindowTimer2, OnRxWindow2TimerEvent );
		TimerInit( &AckTimeoutTimer, OnAckTimeoutTimerEvent );
		
		Channel = LORA_MAX_NB_CHANNELS;// Initialize channel index.
				
#if( OVER_THE_AIR_ACTIVATION == 0 )
		IsLoRaMacNetworkJoined = true;    
#else
    TimerInit( &JoinReqTimer, LoRaMacJoinReq );
    TimerSetValue( &JoinReqTimer, OVER_THE_AIR_ACTIVATION_DUTYCYCLE );
#endif
		ChannelsTxPower = TX_POWER_14_DBM;
		ChannelsMask = 0xFF ;
	
		AdrCtrlOn = true;
		ChannelsDefaultDatarate = ChannelsDatarate = DR_0;	
#if( OVER_THE_AIR_ACTIVATION != 0 )				
		LoRaMacJoinReq();
#endif		
    while(IsLoRaMacNetworkJoined==0);
}

void LoRaMacJoinReq(void)
{
	      uint8_t MHDR=0x00; 
        if( LoRaMacSetNextChannel() == 0 )
        {
						//LoRaMacPrepareFrame
						uint32_t mic = 0;
						LoRaMacBufferPktLen = 0;
						NodeAckRequested = false;
						LoRaMacBuffer[LoRaMacBufferPktLen++] = MHDR;
						RxWindow1Delay = JoinAcceptDelay1 - RADIO_WAKEUP_TIME;
						RxWindow2Delay = JoinAcceptDelay2 - RADIO_WAKEUP_TIME;
						LoRaMacMemCpy( LoRaMacAppEui, LoRaMacBuffer + LoRaMacBufferPktLen, 8 );
						LoRaMacBufferPktLen += 8;
						LoRaMacMemCpy( LoRaMacDevEui, LoRaMacBuffer + LoRaMacBufferPktLen, 8 );
						LoRaMacBufferPktLen += 8;
											
						LoRaMacDevNonce=SX1276Random();										
						LoRaMacBuffer[LoRaMacBufferPktLen++] = LoRaMacDevNonce & 0xFF;
						LoRaMacBuffer[LoRaMacBufferPktLen++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;
						
						AES_CMAC_Init( AesCmacCtx );
						AES_CMAC_SetKey( AesCmacCtx, LoRaMacAppKey );
						AES_CMAC_Update( AesCmacCtx, LoRaMacBuffer, LoRaMacBufferPktLen & 0xFF & 0xFF );
						AES_CMAC_Final( Mic, AesCmacCtx );
						mic = ( uint32_t )( Mic[3] << 24 | Mic[2] << 16 | Mic[1] << 8 | Mic[0] );
						LoRaMacBuffer[LoRaMacBufferPktLen++] = mic & 0xFF;
						LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 8 ) & 0xFF;
						LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 16 ) & 0xFF;
						LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 24 ) & 0xFF;


						LoRaMacEventInfo.TxNbRetries = 0;
						AckReceived = false;
						LoRaMacEventInfo.TxDatarate = Datarates[ChannelsDatarate];
						
						
						if( ChannelsDatarate == DR_6 )// High speed LoRa channel
						{      					
								SX1276_TxConfig( MODEM_LORA, Channels[Channel].Frequency,TxPowers[ChannelsTxPower], 0, 1, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6,&TxTimeOnAir,LoRaMacBufferPktLen );
						}
						else// Normal LoRa channel
						{      
								SX1276_TxConfig( MODEM_LORA, Channels[Channel].Frequency,TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6,&TxTimeOnAir,LoRaMacBufferPktLen );
						}
						if( MaxDCycle == 255 )
								return ;
						if( MaxDCycle == 0 )
								AggregatedTimeOff = 0;
						LoRaMacState |= MAC_TX_RUNNING;
		        TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );						
						TimerEvent_Start( &MacStateCheckTimer );  //Starts the MAC layer status check timer

						if( MAX( Bands[Channels[Channel].Band].TimeOff, AggregatedTimeOff ) > ( TIM2_Time_Present() ))
						{
								// Schedule transmission
								TimerSetValue( &TxDelayedTimer, MAX( Bands[Channels[Channel].Band].TimeOff, AggregatedTimeOff ) );
								TimerEvent_Start( &TxDelayedTimer );
						}
						else
						{
							SX1276_Send( LoRaMacBuffer, LoRaMacBufferPktLen);
						}							
		}
}




uint8_t LoRaMacSendFrameOnChannel( ChannelParams_t channel )
{
        u8 debug_buf[32];
				LoRaMacEventInfo.TxDatarate = Datarates[ChannelsDatarate];
        if( ChannelsDatarate == DR_6 )// High speed LoRa channel
        {       
						SX1276_TxConfig( MODEM_LORA, channel.Frequency, TxPowers[ChannelsTxPower], 0, 1, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 ,&TxTimeOnAir,LoRaMacBufferPktLen );
        }
        else// Normal LoRa channel
        { 
						SX1276_TxConfig( MODEM_LORA, channel.Frequency ,TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 ,&TxTimeOnAir, LoRaMacBufferPktLen);
        }
        if( MaxDCycle == 255 )
						return 6;
        if( MaxDCycle == 0 )
						AggregatedTimeOff = 0;
        LoRaMacState |= MAC_TX_RUNNING;
	    	TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );
        TimerEvent_Start( &MacStateCheckTimer );// Starts the MAC layer status check timer
        if(MAX(Bands[channel.Band].TimeOff,AggregatedTimeOff)>( TIM2_Time_Present() ))// Schedule transmission
        {
            TimerSetValue( &TxDelayedTimer, MAX( Bands[channel.Band].TimeOff, AggregatedTimeOff ) );
            TimerEvent_Start( &TxDelayedTimer );
        }
        else
        {
          SX1276_Send( LoRaMacBuffer, LoRaMacBufferPktLen );				
        }
        return 0;
}

void OnTxDelayedTimerEvent( void )
{
    SX1276_Send( LoRaMacBuffer, LoRaMacBufferPktLen );
}




/*!
 * Function to be executed on Tx Done event
 */
 //static void OnRadioTxDone( void )
void OnRadioTxDone( void )
{
	uint64_t curTime = TIM2_Time_Present();
	SX1276SetSleep();

	// Update Band Time OFF
	Bands[Channels[Channel].Band].LastTxDoneTime = curTime;
	if( DutyCycleOn == true )
	{
			Bands[Channels[Channel].Band].TimeOff = TxTimeOnAir * Bands[Channels[Channel].Band].DCycle - TxTimeOnAir;
	}
	else
	{
			Bands[Channels[Channel].Band].TimeOff = 0;
	}
	// Update Agregated Time OFF
	AggregatedLastTxDoneTime = curTime;
	AggregatedTimeOff = AggregatedTimeOff + ( TxTimeOnAir * AggregatedDCycle - TxTimeOnAir );

	TimerSetValue( &RxWindowTimer1, RxWindow1Delay );
	TimerEvent_Start( &RxWindowTimer1 );
	TimerSetValue( &RxWindowTimer2, RxWindow2Delay );
	TimerEvent_Start( &RxWindowTimer2 );
	if( NodeAckRequested == true )     //如果在第二个接收窗口还未收到ACK，则执行重发
	{
			TimerSetValue( &AckTimeoutTimer, ReceiveDelay2 + ACK_TIMEOUT + randr( -ACK_TIMEOUT_RND, ACK_TIMEOUT_RND ) );
			TimerEvent_Start( &AckTimeoutTimer );
	}	

	if( NodeAckRequested == false )
	{
		ChannelsNbRepCounter++;
	}
}

void OnRadioRxDone( uint8_t *payload, uint16_t size, int8_t rssi, int8_t snr )
{
	uint8_t MHDR;
	uint8_t MHDR_Type;
	uint8_t FCtr;
	uint8_t FOptsLen;	
	uint8_t ACK;
	

  uint8_t pktHeaderLen = 0;
  uint32_t address = 0;
  uint16_t sequenceCounter = 0;
  int32_t sequence = 0;
  uint8_t appPayloadStartIndex = 0;
  uint8_t port = 0xFF;
  uint8_t frameLen = 0;
  uint32_t mic = 0;
  uint32_t micRx = 0;

  MulticastParams_t *curMulticastParams = NULL;
  uint8_t *nwkSKey = LoRaMacNwkSKey;
  uint8_t *appSKey = LoRaMacAppSKey;
  uint32_t downLinkCounter = 0;

  bool isMicOk = false;
  SX1276SetSleep( );

  TimerEvent_Stop( &RxWindowTimer2 );

  MHDR = payload[pktHeaderLen++];
	MHDR_Type=MHDR>>5;

  switch( MHDR_Type )
  {
		case 0x01:     //FRAME_TYPE_JOIN_ACCEPT
          if( IsLoRaMacNetworkJoined == true )
          {
                  break;
          }

					memset1( AesContext.ksch, '\0', 240 );
					aes_set_key( LoRaMacAppKey, 16, &AesContext );
					aes_encrypt( payload + 1, LoRaMacRxPayload + 1, &AesContext );
					// Check if optional CFList is included
					if( size - 1 >= 16 )
					{
							aes_encrypt( payload + 1 + 16, LoRaMacRxPayload + 1 + 16, &AesContext );
					}
          LoRaMacRxPayload[0] = MHDR;

					AES_CMAC_Init( AesCmacCtx );
					AES_CMAC_SetKey( AesCmacCtx, LoRaMacAppKey );
					AES_CMAC_Update( AesCmacCtx, LoRaMacRxPayload, (size - LORAMAC_MFR_LEN) & 0xFF );
					AES_CMAC_Final( Mic, AesCmacCtx );
					mic = ( uint32_t )( Mic[3] << 24 | Mic[2] << 16 | Mic[1] << 8 | Mic[0] );

          micRx |= LoRaMacRxPayload[size - LORAMAC_MFR_LEN];
          micRx |= ( LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 1] << 8 );
          micRx |= ( LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 2] << 16 );
          micRx |= ( LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 3] << 24 );

          if( micRx == mic )
          {
                  LoRaMacEventFlags.Bits.Rx = 1;
                  LoRaMacEventInfo.RxSnr = snr;
                  LoRaMacEventInfo.RxRssi = rssi;

                  //LoRaMacJoinComputeSKeys( LoRaMacAppKey, LoRaMacRxPayload + 1, LoRaMacDevNonce, LoRaMacNwkSKey, LoRaMacAppSKey );
									uint8_t nonce[16];
									uint8_t *pDevNonce = ( uint8_t * )&LoRaMacDevNonce;
									
									memset1( AesContext.ksch, '\0', 240 );
									aes_set_key( LoRaMacAppKey, 16, &AesContext );

									memset1( nonce, 0, sizeof( nonce ) );
									nonce[0] = 0x01;
									LoRaMacMemCpy( LoRaMacRxPayload + 1, nonce + 1, 6 );
									LoRaMacMemCpy( pDevNonce, nonce + 7, 2 );
									aes_encrypt( nonce, LoRaMacNwkSKey, &AesContext );

									memset1( nonce, 0, sizeof( nonce ) );
									nonce[0] = 0x02;
									LoRaMacMemCpy( LoRaMacRxPayload + 1, nonce + 1, 6 );
									LoRaMacMemCpy( pDevNonce, nonce + 7, 2 );
									aes_encrypt( nonce, appSKey, &AesContext );




                  LoRaMacNetID = LoRaMacRxPayload[4];
                  LoRaMacNetID |= ( LoRaMacRxPayload[5] << 8 );
                  LoRaMacNetID |= ( LoRaMacRxPayload[6] << 16 );

                  LoRaMacDevAddr = LoRaMacRxPayload[7];
                  LoRaMacDevAddr |= ( LoRaMacRxPayload[8] << 8 );
                  LoRaMacDevAddr |= ( LoRaMacRxPayload[9] << 16 );
                  LoRaMacDevAddr |= ( LoRaMacRxPayload[10] << 24 );

                  // DLSettings
                  Rx1DrOffset = ( LoRaMacRxPayload[11] >> 4 ) & 0x07;
                  Rx2Channel.Datarate = LoRaMacRxPayload[11] & 0x0F;

                  // RxDelay
                  ReceiveDelay1 = ( LoRaMacRxPayload[12] & 0x0F );
                  if( ReceiveDelay1 == 0 )
                  {
                          ReceiveDelay1 = 1;
                  }
                  ReceiveDelay1 *= 1e6;
                  ReceiveDelay2 = ReceiveDelay1 + 1e6;
                  //ReceiveDelay2 = ReceiveDelay1;

		    /*
                  //CFList
                  if( ( size - 1 ) > 16 )
                  {
                          ChannelParams_t param;
                          param.DrRange.Value = ( DR_5 << 4 ) | DR_0;

                          for( uint8_t i = 3, j = 0; i < ( 5 + 3 ); i++, j += 3 )
                          {
                                  param.Frequency = ( LoRaMacRxPayload[13 + j] | ( LoRaMacRxPayload[14 + j] << 8 ) | ( LoRaMacRxPayload[15 + j] << 16 ) ) * 100;
                                  LoRaMacSetChannel( i, param );
                          }
                  }*/

                  LoRaMacEventFlags.Bits.JoinAccept = 1;
                  IsLoRaMacNetworkJoined = true;
                  ChannelsDatarate = ChannelsDefaultDatarate;
          }
          LoRaMacEventFlags.Bits.Tx = 1;
          break;
  case 0x05:  //FRAME_TYPE_DATA_CONFIRMED_DOWN:
  case 0x03:  //FRAME_TYPE_DATA_UNCONFIRMED_DOWN
  {
			address = payload[pktHeaderLen++];
			address |= ( payload[pktHeaderLen++] << 8 );
			address |= ( payload[pktHeaderLen++] << 16 );
			address |= ( payload[pktHeaderLen++] << 24 );

			if( address != LoRaMacDevAddr ) //地址不匹配
			{
					curMulticastParams = MulticastChannels;
					while( curMulticastParams != NULL )
					{
							if( address == curMulticastParams->Address )
							{
									LoRaMacEventFlags.Bits.Multicast = 1;
									nwkSKey = curMulticastParams->NwkSKey;
									appSKey = curMulticastParams->AppSKey;
									downLinkCounter = curMulticastParams->DownLinkCounter;
									break;
							}
							curMulticastParams = curMulticastParams->Next;
					}
					if( LoRaMacEventFlags.Bits.Multicast == 0 ) // We are not the destination of this frame.
					{	
							LoRaMacEventFlags.Bits.Tx = 1;
							LoRaMacState &= ~MAC_TX_RUNNING;
							return;
					}
			}
			else  //地址匹配
			{
					LoRaMacEventFlags.Bits.Multicast = 0;
					nwkSKey = LoRaMacNwkSKey;
					appSKey = LoRaMacAppSKey;
					downLinkCounter = DownLinkCounter;
			}

			FCtr = payload[pktHeaderLen++];
			FOptsLen=FCtr&0x0F;	
			ACK=(FCtr&0x20)>>5;
			
			sequenceCounter |= payload[pktHeaderLen++];
			sequenceCounter |= payload[pktHeaderLen++] << 8;

			appPayloadStartIndex = 8 + FOptsLen;

			micRx |= payload[size - LORAMAC_MFR_LEN];
			micRx |= ( payload[size - LORAMAC_MFR_LEN + 1] << 8 );
			micRx |= ( payload[size - LORAMAC_MFR_LEN + 2] << 16 );
			micRx |= ( payload[size - LORAMAC_MFR_LEN + 3] << 24 );

			sequence = ( int32_t )sequenceCounter - ( int32_t )( downLinkCounter & 0xFFFF );
			if( sequence < 0 )
			{
							// sequence reset or roll over happened
							downLinkCounter = ( downLinkCounter & 0xFFFF0000 ) | ( sequenceCounter + ( uint32_t )0x10000 );
							//LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounter, &mic );
							//void LoRaMacComputeMic( uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic )
							MicBlockB0[5] = DOWN_LINK;    
							MicBlockB0[6] = ( address ) & 0xFF;
							MicBlockB0[7] = ( address >> 8 ) & 0xFF;
							MicBlockB0[8] = ( address >> 16 ) & 0xFF;
							MicBlockB0[9] = ( address >> 24 ) & 0xFF;
							MicBlockB0[10] = ( downLinkCounter ) & 0xFF;
							MicBlockB0[11] = ( downLinkCounter >> 8 ) & 0xFF;
							MicBlockB0[12] = ( downLinkCounter >> 16 ) & 0xFF;
							MicBlockB0[13] = ( downLinkCounter >> 24 ) & 0xFF;
							MicBlockB0[15] = (size - LORAMAC_MFR_LEN) & 0xFF;
							AES_CMAC_Init( AesCmacCtx );
							AES_CMAC_SetKey( AesCmacCtx, nwkSKey );
							AES_CMAC_Update( AesCmacCtx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE );
							AES_CMAC_Update( AesCmacCtx, payload, size & 0xFF );
							AES_CMAC_Final( Mic, AesCmacCtx );
							mic = ( uint32_t )( Mic[3] << 24 | Mic[2] << 16 | Mic[1] << 8 | Mic[0] );








							if( micRx == mic )
							{
											isMicOk = true;
							}
							else
							{
											isMicOk = false;
											// sequence reset
											if( LoRaMacEventFlags.Bits.Multicast == 1 )
											{
															curMulticastParams->DownLinkCounter = downLinkCounter = sequenceCounter;
											}
											else
											{
															DownLinkCounter = downLinkCounter = sequenceCounter;
											}
											//LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounter, &mic );
											//void LoRaMacComputeMic( uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic )
											MicBlockB0[5] = DOWN_LINK;    
											MicBlockB0[6] = ( address ) & 0xFF;
											MicBlockB0[7] = ( address >> 8 ) & 0xFF;
											MicBlockB0[8] = ( address >> 16 ) & 0xFF;
											MicBlockB0[9] = ( address >> 24 ) & 0xFF;
											MicBlockB0[10] = ( downLinkCounter ) & 0xFF;
											MicBlockB0[11] = ( downLinkCounter >> 8 ) & 0xFF;
											MicBlockB0[12] = ( downLinkCounter >> 16 ) & 0xFF;
											MicBlockB0[13] = ( downLinkCounter >> 24 ) & 0xFF;
											MicBlockB0[15] = (size - LORAMAC_MFR_LEN) & 0xFF;
											AES_CMAC_Init( AesCmacCtx );
											AES_CMAC_SetKey( AesCmacCtx, nwkSKey );
											AES_CMAC_Update( AesCmacCtx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE );
											AES_CMAC_Update( AesCmacCtx, payload, size & 0xFF );
											AES_CMAC_Final( Mic, AesCmacCtx );
											mic = ( uint32_t )( Mic[3] << 24 | Mic[2] << 16 | Mic[1] << 8 | Mic[0] );
							}
			}
			else
			{
							downLinkCounter = ( downLinkCounter & 0xFFFF0000 ) | sequenceCounter;
							//LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounter, &mic );
							//void LoRaMacComputeMic( uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic )
							MicBlockB0[5] = DOWN_LINK;    
							MicBlockB0[6] = ( address ) & 0xFF;
							MicBlockB0[7] = ( address >> 8 ) & 0xFF;
							MicBlockB0[8] = ( address >> 16 ) & 0xFF;
							MicBlockB0[9] = ( address >> 24 ) & 0xFF;
							MicBlockB0[10] = ( downLinkCounter ) & 0xFF;
							MicBlockB0[11] = ( downLinkCounter >> 8 ) & 0xFF;
							MicBlockB0[12] = ( downLinkCounter >> 16 ) & 0xFF;
							MicBlockB0[13] = ( downLinkCounter >> 24 ) & 0xFF;
							MicBlockB0[15] = (size - LORAMAC_MFR_LEN) & 0xFF;
							AES_CMAC_Init( AesCmacCtx );
							AES_CMAC_SetKey( AesCmacCtx, nwkSKey );
							AES_CMAC_Update( AesCmacCtx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE );
							AES_CMAC_Update( AesCmacCtx, payload, size & 0xFF );
							AES_CMAC_Final( Mic, AesCmacCtx );
							mic = ( uint32_t )( Mic[3] << 24 | Mic[2] << 16 | Mic[1] << 8 | Mic[0] );
			}

			if((isMicOk ==true)||(micRx == mic))
			{
					LoRaMacEventFlags.Bits.Rx = 1;
					LoRaMacEventInfo.RxSnr = snr;
					LoRaMacEventInfo.RxRssi = rssi;
					LoRaMacEventInfo.RxBufferSize = 0;
					AdrAckCounter = 0;
					if( LoRaMacEventFlags.Bits.Multicast == 1 )
					{
							curMulticastParams->DownLinkCounter = downLinkCounter;
					}
					else
					{
							DownLinkCounter = downLinkCounter;
					}

					if( MHDR_Type == 0x05 ) //FRAME_TYPE_DATA_CONFIRMED_DOWN
					{
							SrvAckRequested = true;
					}
					else
					{
							SrvAckRequested = false;
					}

					if(ACK == 1 )  //ACK frame 
					{
							AckReceived = true;
							TimerEvent_Stop( &AckTimeoutTimer );
					}
					else           //not ACK frame
					{
							AckReceived = false;
							if( AckTimeoutRetriesCounter > AckTimeoutRetries )
							{
											TimerEvent_Stop( &AckTimeoutTimer );
							}
					}

					if(FOptsLen > 0 )
					{
								// Decode Options field MAC commands
								// LoRaMacProcessMacCommands( payload, 8, appPayloadStartIndex );
								//static void LoRaMacProcessMacCommands( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize )
							while( 8 < appPayloadStartIndex )
							{
									// Decode Frame MAC commands
									switch( payload[8] )
									{
									case SRV_MAC_LINK_CHECK_ANS:
											LoRaMacEventFlags.Bits.LinkCheck = 1;
											LoRaMacEventInfo.DemodMargin = payload[9];
											LoRaMacEventInfo.NbGateways = payload[10];
									break;
									case SRV_MAC_LINK_ADR_REQ:
									{
											uint8_t i;
											uint8_t status = 0x07;
											uint16_t chMask = 0;
											int8_t txPower = 0;
											int8_t datarate = 0;
											uint8_t nbRep = 0;
											uint8_t chMaskCntl = 0;
											datarate = payload[11];
											txPower = datarate & 0x0F;
											datarate = ( datarate >> 4 ) & 0x0F;
											if((AdrCtrlOn==false)&&((ChannelsDatarate!=datarate)||(ChannelsTxPower!=txPower)))
											{
															// ADR disabled don't handle ADR requests if server tries to change datarate or txpower
															// Answer the server with fail status
															// Powe   = 0
															// Data rate ACr ACK  K = 0
															// Channel mask  = 0
															AddMacCommand( MOTE_MAC_LINK_ADR_ANS, 0, 0 );
															break;
											}
											chMask = payload[12];
											chMask |= payload[13] << 8;
											nbRep = payload[14];
											chMaskCntl = ( nbRep >> 4 ) & 0x07;
											nbRep &= 0x0F;
											if( nbRep == 0 )
													nbRep = 1;
											if( ( chMaskCntl == 0 ) && ( chMask == 0 ) )
													status &= 0xFE; // Channel mask KO
											else
											{
													for( i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
													{
														if( chMaskCntl == 6 )
														{
																		if( Channels[i].Frequency != 0 )
																						chMask |= 1 << i;
														}
														else
														{
																		if(((chMask&(1<<i))!= 0)&&(Channels[i].Frequency == 0 ) )
																						status &= 0xFE; // Channel mask KO  Trying to enable an undefined channel
														}
													}
											}
											if(((datarate < LORAMAC_MIN_DATARATE)||(datarate>LORAMAC_MAX_DATARATE))==true)
															status &= 0xFD; // Datarate KO

											// Remark MaxTxPower = 0 and MinTxPower = 5
											if(((LORAMAC_MAX_TX_POWER <= txPower)&&(txPower<=LORAMAC_MIN_TX_POWER))==false)
															status &= 0xFB; // TxPower KO
											if((status & 0x07 ) == 0x07 )
											{
															ChannelsDatarate = datarate;
															ChannelsTxPower = txPower;
															ChannelsMask = chMask;
															ChannelsNbRep = nbRep;
											}
											AddMacCommand( MOTE_MAC_LINK_ADR_ANS, status, 0 );
									}
									break;
									case SRV_MAC_DUTY_CYCLE_REQ:
											MaxDCycle = payload[15];
											AggregatedDCycle = 1 << MaxDCycle;
											AddMacCommand( MOTE_MAC_DUTY_CYCLE_ANS, 0, 0 );
									break;
									case SRV_MAC_RX_PARAM_SETUP_REQ:
									{
											uint8_t status = 0x07;
											int8_t datarate = 0;
											int8_t drOffset = 0;
											uint32_t freq = 0;

											drOffset = payload[16];
											datarate = drOffset & 0x0F;
											drOffset = ( drOffset >> 4 ) & 0x0F;

											freq = payload[17];
											freq |= payload[18] << 8;
											freq |= payload[19] << 16;
											freq *= 100;

											if( ( ( datarate < LORAMAC_MIN_DATARATE ) ||( datarate > LORAMAC_MAX_DATARATE ) ) == true )
													status &= 0xFD; // Datarate KO
											if( ( ( drOffset < 0 ) || ( drOffset > 5 ) ) == true )
													status &= 0xFB; // Rx1DrOffset range KO
											if( ( status & 0x07 ) == 0x07 )
											{
													Rx2Channel.Datarate = datarate;
													Rx2Channel.Frequency = freq;
													Rx1DrOffset = drOffset;
											}
											AddMacCommand( MOTE_MAC_RX_PARAM_SETUP_ANS, status, 0 );
									}
									break;
									case SRV_MAC_DEV_STATUS_REQ:
											AddMacCommand( MOTE_MAC_DEV_STATUS_ANS, BoardMeasureBatterieLevel( ), LoRaMacEventInfo.RxSnr );
											break;
									case SRV_MAC_NEW_CHANNEL_REQ:
									{
											uint8_t status = 0x03;
											int8_t channelIndex = 0;
											ChannelParams_t chParam;
											channelIndex = payload[20];
											chParam.Frequency = payload[21];
											chParam.Frequency |= payload[22] << 8;
											chParam.Frequency |= payload[23] << 16;
											chParam.Frequency *= 100;
											chParam.DrRange.Value = payload[24];
											if((channelIndex<3)||( channelIndex>LORA_MAX_NB_CHANNELS))
															status &= 0xFE; // Channel frequency KO
											if((chParam.DrRange.Fields.Min > chParam.DrRange.Fields.Max )||(((LORAMAC_MIN_DATARATE <=chParam.DrRange.Fields.Min)&&(chParam.DrRange.Fields.Min <= LORAMAC_MAX_DATARATE))==false)||(((LORAMAC_MIN_DATARATE <= chParam.DrRange.Fields.Max)&&(chParam.DrRange.Fields.Max <= LORAMAC_MAX_DATARATE))==false))
															status &= 0xFD; // Datarate range KO
											if( ( status & 0x03 ) == 0x03 )
											{
													chParam.Band = 0;
													Channels[channelIndex] = chParam;
													// Activate the newly created channel
													ChannelsMask |= 1 << channelIndex;
													if( Channels[channelIndex].Frequency == 0 )
																	ChannelsMask &= ~( 1 << channelIndex );			
											}
											AddMacCommand( MOTE_MAC_NEW_CHANNEL_ANS, status, 0 );
									}
									break;
									case SRV_MAC_RX_TIMING_SETUP_REQ:
									{
											uint8_t delay = payload[25] & 0x0F;
											if( delay == 0 )
															delay++;
											ReceiveDelay1 = delay * 1e6;
											ReceiveDelay2 = ReceiveDelay1 + 1e6;
											AddMacCommand( MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0 );
									}
									break;
									default:// Unknown command. ABORT MAC commands processing
											return;
									}
							}									
					}

					if(((size-4)-appPayloadStartIndex)>0)
					{
							port = payload[appPayloadStartIndex++];
							frameLen = ( size - 4 ) - appPayloadStartIndex;
							if( port == 0 )
							{
										//LoRaMacPayloadEncrypt( payload + appPayloadStartIndex, frameLen, nwkSKey, address, DOWN_LINK, downLinkCounter, LoRaMacRxPayload );( uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer )
										// Decode frame payload MAC commands
										uint16_t i;
										uint8_t bufferIndex = 0;
										uint16_t ctr = 1;
										memset1( AesContext.ksch, '\0', 240 );
										aes_set_key( nwkSKey, 16, &AesContext );
										aBlock[5] = DOWN_LINK;
										aBlock[6] = ( address ) & 0xFF;
										aBlock[7] = ( address >> 8 ) & 0xFF;
										aBlock[8] = ( address >> 16 ) & 0xFF;
										aBlock[9] = ( address >> 24 ) & 0xFF;
										aBlock[10] = ( downLinkCounter ) & 0xFF;
										aBlock[11] = ( downLinkCounter >> 8 ) & 0xFF;
										aBlock[12] = ( downLinkCounter >> 16 ) & 0xFF;
										aBlock[13] = ( downLinkCounter >> 24 ) & 0xFF;
										while( frameLen >= 16 )
										{
												aBlock[15] = ( ( ctr ) & 0xFF );
												ctr++;
												aes_encrypt( aBlock, sBlock, &AesContext );
												for( i = 0; i < 16; i++ )
														LoRaMacRxPayload[bufferIndex+i]=*(payload+appPayloadStartIndex+bufferIndex+i)^sBlock[i];
												frameLen -= 16;
												bufferIndex += 16;
										}
										if( frameLen > 0 )
										{
												aBlock[15] = ( ( ctr ) & 0xFF );
												aes_encrypt( aBlock, sBlock, &AesContext );
												for( i = 0; i < frameLen; i++ )
														LoRaMacRxPayload[bufferIndex+i]=*(payload+appPayloadStartIndex+bufferIndex+i)^sBlock[i];
										}

								// LoRaMacProcessMacCommands( LoRaMacRxPayload, 0, frameLen );
										while( 0 < frameLen )
										{
												switch( payload[0] )// Decode Frame MAC commands
												{
												case SRV_MAC_LINK_CHECK_ANS:
														LoRaMacEventFlags.Bits.LinkCheck = 1;
														LoRaMacEventInfo.DemodMargin = payload[1];
														LoRaMacEventInfo.NbGateways = payload[2];
												break;
												case SRV_MAC_LINK_ADR_REQ:
												{
														uint8_t i;
														uint8_t status = 0x07;
														uint16_t chMask = 0;
														int8_t txPower = 0;
														int8_t datarate = 0;
														uint8_t nbRep = 0;
														uint8_t chMaskCntl = 0;

														datarate = payload[3];
														txPower = datarate & 0x0F;
														datarate = ( datarate >> 4 ) & 0x0F;

														if((AdrCtrlOn ==false)&&((ChannelsDatarate!=datarate )||(ChannelsTxPower!=txPower)))
														{
																		// ADR disabled don't handle ADR requests if server tries to change datarate or txpower
																		// Answer the server with fail status
																		// Powe   = 0
																		// Data rate ACr ACK  K = 0
																		// Channel mask  = 0
																		AddMacCommand( MOTE_MAC_LINK_ADR_ANS, 0, 0 );
																		break;
														}
														chMask = payload[4];
														chMask |= payload[5] << 8;
														nbRep = payload[6];
														chMaskCntl = ( nbRep >> 4 ) & 0x07;
														nbRep &= 0x0F;
														if( nbRep == 0 )
																		nbRep = 1;
														if( ( chMaskCntl == 0 ) && ( chMask == 0 ) )
																		status &= 0xFE; // Channel mask KO
														else
														{
																for( i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
																{
																		if( chMaskCntl == 6 )
																		{
																						if( Channels[i].Frequency != 0 )
																										chMask |= 1 << i;
																		}
																		else
																		{
																						if( ( ( chMask & ( 1 << i ) ) != 0 ) &&( Channels[i].Frequency == 0 ) )
																										status &= 0xFE; // Channel mask KO // Trying to enable an undefined channel
																		}
																}
														}
														if( ( ( datarate < LORAMAC_MIN_DATARATE ) ||( datarate > LORAMAC_MAX_DATARATE ) ) == true )
																		status &= 0xFD; // Datarate KO
														if(((LORAMAC_MAX_TX_POWER <= txPower )&&(txPower <= LORAMAC_MIN_TX_POWER ) ) == false )// Remark MaxTxPower = 0 and MinTxPower = 5
																		status &= 0xFB; // TxPower KO
														if((status&0x07)==0x07)
														{
																		ChannelsDatarate = datarate;
																		ChannelsTxPower = txPower;
																		ChannelsMask = chMask;
																		ChannelsNbRep = nbRep;
														}
														AddMacCommand( MOTE_MAC_LINK_ADR_ANS, status, 0 );
												}
												break;
												case SRV_MAC_DUTY_CYCLE_REQ:
														MaxDCycle = payload[7];
														AggregatedDCycle = 1 << MaxDCycle;
														AddMacCommand( MOTE_MAC_DUTY_CYCLE_ANS, 0, 0 );
												break;
												case SRV_MAC_RX_PARAM_SETUP_REQ:
												{
														uint8_t status = 0x07;
														int8_t datarate = 0;
														int8_t drOffset = 0;
														uint32_t freq = 0;

														drOffset = payload[8];
														datarate = drOffset & 0x0F;
														drOffset = ( drOffset >> 4 ) & 0x0F;

														freq = payload[9];
														freq |= payload[10] << 8;
														freq |= payload[11] << 16;
														freq *= 100;

														if( ( ( datarate < LORAMAC_MIN_DATARATE ) ||( datarate > LORAMAC_MAX_DATARATE ) ) == true )
																		status &= 0xFD; // Datarate KO
														if( ( ( drOffset < 0 ) || ( drOffset > 5 ) ) == true )
																		status &= 0xFB; // Rx1DrOffset range KO
														if( ( status & 0x07 ) == 0x07 )
														{
																		Rx2Channel.Datarate = datarate;
																		Rx2Channel.Frequency = freq;
																		Rx1DrOffset = drOffset;
														}
														AddMacCommand( MOTE_MAC_RX_PARAM_SETUP_ANS, status, 0 );
												}
												break;
												case SRV_MAC_DEV_STATUS_REQ:
														AddMacCommand( MOTE_MAC_DEV_STATUS_ANS, BoardMeasureBatterieLevel( ), LoRaMacEventInfo.RxSnr );
												break;
												case SRV_MAC_NEW_CHANNEL_REQ:
												{
														uint8_t status = 0x03;
														int8_t channelIndex = 0;
														ChannelParams_t chParam;
														channelIndex = payload[12];
														chParam.Frequency = payload[13];
														chParam.Frequency |= payload[14] << 8;
														chParam.Frequency |= payload[15] << 16;
														chParam.Frequency *= 100;
														chParam.DrRange.Value = payload[16];
														if( ( channelIndex < 3 ) || ( channelIndex > LORA_MAX_NB_CHANNELS ) )
																		status &= 0xFE; // Channel frequency KO
														if((chParam.DrRange.Fields.Min > chParam.DrRange.Fields.Max )||(((LORAMAC_MIN_DATARATE <= chParam.DrRange.Fields.Min )&&(chParam.DrRange.Fields.Min <= LORAMAC_MAX_DATARATE ) ) == false ) ||(((LORAMAC_MIN_DATARATE <= chParam.DrRange.Fields.Max ) &&( chParam.DrRange.Fields.Max <= LORAMAC_MAX_DATARATE ) ) == false ) )
																		status &= 0xFD; // Datarate range KO
														if( ( status & 0x03 ) == 0x03 )
														{
																chParam.Band = 0;
																Channels[channelIndex] = chParam;
																// Activate the newly created channel
																ChannelsMask |= 1 << channelIndex;
																if( Channels[channelIndex].Frequency == 0 )
																				ChannelsMask &= ~( 1 << channelIndex );			
														}
														AddMacCommand( MOTE_MAC_NEW_CHANNEL_ANS, status, 0 );
												}
												break;
												case SRV_MAC_RX_TIMING_SETUP_REQ:
												{
														uint8_t delay = payload[17] & 0x0F;
														if( delay == 0 )
																		delay++;
														ReceiveDelay1 = delay * 1e6;
														ReceiveDelay2 = ReceiveDelay1 + 1e6;
														AddMacCommand( MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0 );
												}
												break;
												default:	
														return;// Unknown command. ABORT MAC commands processing
												}
										}
							}
							else
							{
									//LoRaMacPayloadEncrypt( payload+appPayloadStartIndex, frameLen, appSKey, address, DOWN_LINK, downLinkCounter, LoRaMacRxPayload );( uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer )
									uint16_t i;
									uint8_t bufferIndex = 0;
									uint16_t ctr = 1;
									memset1( AesContext.ksch, '\0', 240 );
									aes_set_key( appSKey, 16, &AesContext );
									aBlock[5] =DOWN_LINK;
									aBlock[6] = ( address ) & 0xFF;
									aBlock[7] = ( address >> 8 ) & 0xFF;
									aBlock[8] = ( address >> 16 ) & 0xFF;
									aBlock[9] = ( address >> 24 ) & 0xFF;
									aBlock[10] = ( downLinkCounter ) & 0xFF;
									aBlock[11] = ( downLinkCounter >> 8 ) & 0xFF;
									aBlock[12] = ( downLinkCounter >> 16 ) & 0xFF;
									aBlock[13] = ( downLinkCounter >> 24 ) & 0xFF;
									while( frameLen >= 16 )
									{
											aBlock[15] = ( ( ctr ) & 0xFF );
											ctr++;
											aes_encrypt( aBlock, sBlock, &AesContext );
											for( i = 0; i < 16; i++ )
													LoRaMacRxPayload[bufferIndex+i]=*(payload+appPayloadStartIndex+bufferIndex+i)^sBlock[i];
											frameLen -= 16;
											bufferIndex += 16;
									}
									if( frameLen > 0 )
									{
											aBlock[15] = ( ( ctr ) & 0xFF );
											aes_encrypt( aBlock, sBlock, &AesContext );
											for( i = 0; i < frameLen; i++ )
													LoRaMacRxPayload[bufferIndex+i]=*(payload+appPayloadStartIndex+bufferIndex+i)^sBlock[i];
									}

									
									LoRaMacEventFlags.Bits.RxData = 1;
									LoRaMacEventInfo.RxPort = port;
									LoRaMacEventInfo.RxBuffer = LoRaMacRxPayload;
									LoRaMacEventInfo.RxBufferSize = frameLen;
							}
					}

					LoRaMacEventFlags.Bits.Tx = 1;
			}
			else
			{
					AckReceived = false;
					LoRaMacEventFlags.Bits.Tx = 1;
					LoRaMacState &= ~MAC_TX_RUNNING;
			}
  }
  break;
	case 0x07:    //FRAME_TYPE_PROPRIETARY:
  //Intentional falltrough
  default:
			LoRaMacEventFlags.Bits.Tx = 1;
			LoRaMacState &= ~MAC_TX_RUNNING;
	break;
  }
}

void OnRadioTxTimeout( void )
{
		SX1276SetSleep( );
		LoRaMacEventFlags.Bits.Tx = 1;
}




void OnRadioRxTimeout( void )
{
		SX1276SetSleep();
		if( LoRaMacEventFlags.Bits.RxSlot == 1 )
		{
				LoRaMacEventFlags.Bits.Tx = 1;
		}
}

void OnRadioRxError( void )
{

         SX1276SetSleep( );
        if( LoRaMacEventFlags.Bits.RxSlot == 1 )
        {
                LoRaMacEventFlags.Bits.Tx = 1;
        }
}

void OnRadioFhssChangeChannel( uint8_t currentChannel )
{}
void OnRadioCadDone( bool channelActivityDetected )
{}

	

/*!
 * Initializes and opens the reception window
 *
 * \param [IN] freq window channel frequency
 * \param [IN] datarate window channel datarate
 * \param [IN] bandwidth window channel bandwidth
 * \param [IN] timeout window channel timeout
 */


/*!
 * Function executed on first Rx window timer event
 */
static void OnRxWindow1TimerEvent( void )
{
        uint16_t symbTimeout = 5; // DR_2, DR_1, DR_0
        int8_t datarate = 0;
        uint32_t bandwidth = 0; // LoRa 125 kHz

        datarate = ChannelsDatarate - Rx1DrOffset;
        if( datarate < 0 )
        {
						datarate = DR_0;
        }
        // For higher datarates, we increase the number of symbols generating a Rx Timeout
        if( datarate >= DR_3 )
        {
                // DR_6, DR_5, DR_4, DR_3
                symbTimeout = 8;
        }
        if( datarate == DR_6 )
        {
                // LoRa 250 kHz
                bandwidth  = 1;
        }
        LoRaMacEventFlags.Bits.RxSlot = 0;

        //LoRaMacRxWindowSetup( Channels[Channel].Frequency, datarate, bandwidth, symbTimeout, false );{
        if( SX1276_State == RF_IDLE )
        {
						SX1276_RxConfig( MODEM_LORA,DnChannels[Channel].Frequency, bandwidth, Datarates[datarate], 1, 0, 8, symbTimeout, false, 0, false, 0, 0, true, false,MAX_RX_WINDOW );
 				}
}

/*!
 * Function executed on second Rx window timer event
 */
static void OnRxWindow2TimerEvent( void )
{
        uint16_t symbTimeout = 5; // DR_2, DR_1, DR_0
        uint32_t bandwidth = 0; // LoRa 125 kHz

        // For higher datarates, we increase the number of symbols generating a Rx Timeout
        if( Rx2Channel.Datarate >= DR_3 )
        {
                // DR_6, DR_5, DR_4, DR_3
						symbTimeout = 8;
        }
        if( Rx2Channel.Datarate == DR_6 )
        {
                // LoRa 250 kHz
						bandwidth  = 1;
        }

        LoRaMacEventFlags.Bits.RxSlot = 1;
			 //LoRaMacRxWindowSetup( Rx2Channel.Frequency, Rx2Channel.Datarate, bandwidth, symbTimeout, false );
				if( SX1276_State == RF_IDLE )
				{
						SX1276_RxConfig( MODEM_LORA,Rx2Channel.Frequency,bandwidth, Datarates[Rx2Channel.Datarate], 1, 0, 8, symbTimeout, false, 0, false, 0, 0, true, false,MAX_RX_WINDOW );         
				}


}

/*!
 * Function executed on MacStateCheck timer event
 */
static void OnMacStateCheckTimerEvent( void )
{
	if( LoRaMacEventFlags.Bits.Tx == 1 )
	{
			if( NodeAckRequested == false )
			{
							if( LoRaMacEventFlags.Bits.JoinAccept == true )
							{
									// Join messages aren't repeated automatically
									ChannelsNbRepCounter = ChannelsNbRep;
									UpLinkCounter = 0;
							}
							if( ChannelsNbRepCounter >= ChannelsNbRep )
							{
									ChannelsNbRepCounter = 0;
									AdrAckCounter++;
									if( IsUpLinkCounterFixed == false )
									{
													UpLinkCounter++;
									}
									LoRaMacState &= ~MAC_TX_RUNNING;
							}
							else
							{
									LoRaMacEventFlags.Bits.Tx = 0;
									if( LoRaMacSetNextChannel() == 0 )// Sends the same frame again
									{
													LoRaMacSendFrameOnChannel( Channels[Channel] );
									}
							}
			}

			if( LoRaMacEventFlags.Bits.Rx == 1 )
			{
					if( ( AckReceived == true ) || ( AckTimeoutRetriesCounter > AckTimeoutRetries ) )
					{
							AckTimeoutRetry = false;
							if( IsUpLinkCounterFixed == false )
							{
											UpLinkCounter++;
							}
							LoRaMacEventInfo.TxNbRetries = AckTimeoutRetriesCounter;

							LoRaMacState &= ~MAC_TX_RUNNING;
					}
			}
			if((AckTimeoutRetry==true) && ((LoRaMacState&MAC_CHANNEL_CHECK) ==0))
			{
					AckTimeoutRetry = false;
					if( AckTimeoutRetriesCounter < AckTimeoutRetries )  //重发次数未达上限
					{
							AckTimeoutRetriesCounter++;          
							if((AckTimeoutRetriesCounter%2) == 1 )     //设置发送速率略小于上次发送速率
									ChannelsDatarate = MAX( ChannelsDatarate - 1, LORAMAC_MIN_DATARATE );
							LoRaMacEventFlags.Bits.Tx = 0;
							if( LoRaMacSetNextChannel() == 0 )
							{
									LoRaMacSendFrameOnChannel( Channels[Channel] );
							}
					}    
					else                                                //重发次数达到上限
					{
									ChannelsMask = ChannelsMask | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );// Re-enable default channels LC1, LC2, LC3
									LoRaMacState &= ~MAC_TX_RUNNING;
									AckReceived = false;                      //本次发送的结果：未收到ACK 
									LoRaMacEventInfo.TxNbRetries = AckTimeoutRetriesCounter; //本次发送的重发次数
					}
			}
	}
	// Handle reception for Class B and Class C
	if( ( LoRaMacState & MAC_RX ) == MAC_RX )
	{
			LoRaMacState &= ~MAC_RX;
	}
	if( LoRaMacState == MAC_IDLE )
	{
			LoRaMacEventFlags.Value = 0;		
	}
	else
	{
			TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );		
			TimerEvent_Start( &MacStateCheckTimer );// Operation not finished restart timer
	}
}

static void OnAckTimeoutTimerEvent( void )
{
        AckTimeoutRetry = true;
        LoRaMacState &= ~MAC_ACK_REQ;
}

//MHDR：MType(3)+RFU(3)+Major(2)
//FCtr：ADR+ADRACKReq+ACK+FPending+FOptsLen(4)
void LoRaMac_Send(uint8_t *fBuffer, uint16_t fBufferSize, bool confirmed )
{
	uint8_t MHDR;
	if(confirmed==0)
	{
		MHDR=0x40;  //uncomfirmed data up
    NodeAckRequested = false;
	}
	else
	{
		MHDR=0x80;  //uncomfirmed data up
		NodeAckRequested = true;		
	}
		
	uint8_t FCtr=0;
	uint8_t FOptsLen=FCtr&0x0F;
	uint16_t FCnt=UpLinkCounter;
	uint8_t *FOpts=NULL;
	uint8_t FPort=2;
	uint32_t MIC = 0;
	
  uint64_t nextTxDelay = 0xFFFFFFFFFFFFFFFF;
	LoRaMacBufferPktLen = 0;	
	uint64_t curTime = TIM2_Time_Present();

	uint8_t bufferIndex = 0;
	uint16_t ctr = 1;	

	RxWindow1Delay = ReceiveDelay1 - RADIO_WAKEUP_TIME;
	RxWindow2Delay = ReceiveDelay2 - RADIO_WAKEUP_TIME;

	LoRaMacBuffer[LoRaMacBufferPktLen++] = MHDR;		
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( LoRaMacDevAddr ) & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( LoRaMacDevAddr >> 24 ) & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = FCtr;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = FCnt & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( FCnt >> 8 ) & 0xFF;
	if( FOpts != NULL )
	{
		for(uint8_t i=0; i<FOptsLen; i++ )
			LoRaMacBuffer[LoRaMacBufferPktLen++] = FOpts[i];
	}	
	LoRaMacBuffer[LoRaMacBufferPktLen++] = FPort;

	//LoRaMacPayloadEncrypt
	memset1( AesContext.ksch, '\0', 240 );
	aes_set_key( LoRaMacAppSKey, 16, &AesContext );
	aBlock[5] = UP_LINK;
	aBlock[6] = ( LoRaMacDevAddr ) & 0xFF;
	aBlock[7] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
	aBlock[8] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
	aBlock[9] = ( LoRaMacDevAddr >> 24 ) & 0xFF;
	aBlock[10] = ( UpLinkCounter ) & 0xFF;
	aBlock[11] = ( UpLinkCounter >> 8 ) & 0xFF;
	aBlock[12] = ( UpLinkCounter >> 16 ) & 0xFF;
	aBlock[13] = ( UpLinkCounter >> 24 ) & 0xFF;
	while( fBufferSize >= 16 )
	{
			aBlock[15] = ( ( ctr ) & 0xFF );
			ctr++;
			aes_encrypt( aBlock, sBlock, &AesContext );
			for( uint16_t i = 0; i < 16; i++ )
					LoRaMacPayload[bufferIndex + i] = fBuffer[bufferIndex + i] ^ sBlock[i];
			fBufferSize -= 16;
			bufferIndex += 16;
	}

	if( fBufferSize > 0 )
	{
			aBlock[15] = ( ( ctr ) & 0xFF );
			aes_encrypt( aBlock, sBlock, &AesContext );
			for( uint16_t i = 0; i < fBufferSize; i++ )
					LoRaMacPayload[bufferIndex + i] = fBuffer[bufferIndex + i] ^ sBlock[i];
	}


	LoRaMacMemCpy( LoRaMacPayload, LoRaMacBuffer + LoRaMacBufferPktLen, fBufferSize );
	LoRaMacBufferPktLen = LoRaMacBufferPktLen + fBufferSize;

	//LoRaMacComputeMic( LoRaMacBuffer, LoRaMacBufferPktLen, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &MIC );
	MicBlockB0[5] = UP_LINK;
	MicBlockB0[6] = ( LoRaMacDevAddr ) & 0xFF;
	MicBlockB0[7] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
	MicBlockB0[8] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
	MicBlockB0[9] = ( LoRaMacDevAddr >> 24 ) & 0xFF;
	MicBlockB0[10] = ( UpLinkCounter ) & 0xFF;
	MicBlockB0[11] = ( UpLinkCounter >> 8 ) & 0xFF;
	MicBlockB0[12] = ( UpLinkCounter >> 16 ) & 0xFF;
	MicBlockB0[13] = ( UpLinkCounter >> 24 ) & 0xFF;
	MicBlockB0[15] = LoRaMacBufferPktLen & 0xFF;
	AES_CMAC_Init( AesCmacCtx );
	AES_CMAC_SetKey( AesCmacCtx, LoRaMacNwkSKey );
	AES_CMAC_Update( AesCmacCtx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE );
	AES_CMAC_Update( AesCmacCtx, LoRaMacBuffer, LoRaMacBufferPktLen & 0xFF );
	AES_CMAC_Final( Mic, AesCmacCtx );
	MIC = ( uint32_t )( Mic[3] << 24 | Mic[2] << 16 | Mic[1] << 8 | Mic[0] );				
	
	LoRaMacBuffer[LoRaMacBufferPktLen++] = MIC & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( MIC >> 8 ) & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( MIC >> 16 ) & 0xFF;
	LoRaMacBuffer[LoRaMacBufferPktLen++] = ( MIC >> 24 ) & 0xFF;	



	if( MaxDCycle == 255 )
			return;
	if( MaxDCycle == 0 )
			AggregatedTimeOff = 0;	

	
	//本次发送距离上次发送，间隔时间超过允许值，则可以发送
	if( AggregatedTimeOff <= ( curTime - AggregatedLastTxDoneTime ) )
	{
		  ChannelParams_t channel;
			uint8_t i;
			AggregatedTimeOff = 0;
			for( i = 0; i < LORA_MAX_NB_BANDS; i++ )
			{
				if( DutyCycleOn == true )
				{
						if( Bands[i].TimeOff < ( curTime - Bands[i].LastTxDoneTime ) )
						{
								Bands[i].TimeOff = 0;
						}
						if( Bands[i].TimeOff != 0 )
						{
								nextTxDelay = MIN( Bands[i].TimeOff, nextTxDelay );
						}
				}
				else
				{
						nextTxDelay = 0;
						Bands[i].TimeOff = 0;
				}
			}		
			Channel= randr( 0, LORA_MAX_NB_CHANNELS - 1 );
			LoRaMacState &= ~MAC_CHANNEL_CHECK;  //找到信道，退出查找信道的状态
			TimerEvent_Stop( &ChannelCheckTimer );			
			channel=Channels[Channel];	
			
			LoRaMacEventInfo.TxNbRetries = 0;    //重发次数初始化，使用LoRaMac_Send时，必定一次都没重发
			AckReceived = false;
			LoRaMacEventInfo.TxDatarate = Datarates[ChannelsDatarate];
			
			SX1276_TxConfig( MODEM_LORA, channel.Frequency,TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 , &TxTimeOnAir, LoRaMacBufferPktLen );// Normal LoRa channel
			LoRaMacState |= MAC_TX_RUNNING;
		  TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );	
		  //TimerEvent_Start( &MacStateCheckTimer );// Starts the MAC layer status check timer

			SX1276_Send(LoRaMacBuffer, LoRaMacBufferPktLen);	
			UpLinkCounter++;		
    		
	}

	//本次发送距离上次发送，间隔时间小于允许值，推迟相应的时间再发送
	else if( ( LoRaMacState & MAC_CHANNEL_CHECK ) == 0 )
	{
		nextTxDelay = AggregatedTimeOff - ( curTime - AggregatedLastTxDoneTime );
		TimerSetValue( &ChannelCheckTimer, nextTxDelay );
		TimerEvent_Start( &ChannelCheckTimer );
		LoRaMacState |= MAC_CHANNEL_CHECK;
	}
}



