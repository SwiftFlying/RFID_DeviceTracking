#include "sx1276.h"
#include "LoRaMac.h"

//结点参数
uint32_t LoRaMacDevAddr = 0x28AD9150 ;
uint8_t LoRaMacNwkSKey[] ={0x3a, 0x1e, 0xbd, 0x00, 0x3a, 0xc8, 0x8c, 0x00, 0xc0, 0xc5, 0x1e, 0xff, 0xea, 0xd2, 0x65, 0x50  };
uint8_t LoRaMacAppSKey[] ={0xe0, 0x8c, 0x28, 0xff, 0xcf, 0x7a, 0x47, 0xff, 0x7a, 0xb5, 0xeb, 0x00, 0xbd, 0x00, 0x01, 0x50  };
uint8_t LoRaMacDevEui[] ={0x70, 0xb3, 0xd5, 0x31, 0xc0, 0x00, 0x01, 0x50 };  
uint8_t LoRaMacAppEui[] ={0x70,0xb3,0xd5,0x31,0xc0,0x00,0x00,0x01};          
uint8_t LoRaMacAppKey[]={0xb3,0x19,0x0f,0xff,0x13,0xd6,0x93,0x00,0xad,0xa1,0x09,0xff,0x24,0x3e,0xa5,0x01 };
uint32_t LoRaMacNetID = 0xAA5E1A;

/*
//ZTE
uint32_t LoRaMacDevAddr = 0x28AD9151 ;
uint8_t LoRaMacNwkSKey[] ={0x3a, 0x1e, 0xbd, 0x00, 0x3a, 0xc8, 0x8c, 0x00, 0xc0, 0xc5, 0x1e, 0xff, 0xea, 0xd2, 0x65, 0x51  };//kiwi
uint8_t LoRaMacAppSKey[] ={0xe0, 0x8c, 0x28, 0xff, 0xcf, 0x7a, 0x47, 0xff, 0x7a, 0xb5, 0xeb, 0x00, 0xbd, 0x00, 0x01, 0x51  };//KIWI
uint8_t LoRaMacDevEui[] ={0x01,0x00,0x11,0xab,0x00,0x77,0x4a,0x00 };//ZTE
uint8_t LoRaMacAppEui[] ={0x11, 0x00, 0x00, 0xab, 0x00, 0xc5, 0x26, 0x2c  };//ZTE
uint8_t LoRaMacAppKey[]={0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };//for ZTE
uint32_t LoRaMacNetID = 0xAA5E1A;
*/



//define TxPower ，发送功率
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };   //存储在flash中，因为在接收时会使用到
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
    DrRange_t DrRange;  // 高4位最大值，低4位最小值
    uint8_t Band;       // Band index
}PACKED ChannelParams_t;

typedef struct
{
    uint32_t Frequency; // Hz
    uint8_t  Datarate;  // [0: SF12, 1: SF11, 2: SF10, 3: SF9, 4: SF8, 5: SF7, 6: SF7, 7: FSK]
}PACKED Rx2ChannelParams_t;



//通道


#define LORA_MAX_NB_CHANNELS  3 
ChannelParams_t LoRaMac_Channel_UpLink[LORA_MAX_NB_CHANNELS] =
{
{ 470300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 470500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 470700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }, 
};
ChannelParams_t LoRaMac_Channel_DownLink[LORA_MAX_NB_CHANNELS]=
{
{ 500300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
{ 500500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },          
{ 500700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },      
};

/*
//ZTE
#define LORA_MAX_NB_CHANNELS  16 
ChannelParams_t LoRaMac_Channel_UpLink[LORA_MAX_NB_CHANNELS]=
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
ChannelParams_t LoRaMac_Channel_DownLink[16]=
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

static Rx2ChannelParams_t Rx2Channel = {471725000, DR_0}; //LoRaMAC 2nd reception window settings
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
static uint16_t ChannelsMask;   //Mask indicating which channels are enabled
static uint8_t ChannelsNbRep = 1; //Number of uplink messages repetitions [1:15] (unconfirmed messages only)
static uint8_t ChannelsNbRepCounter = 0; //Uplink messages repetitions counter重复接收
static uint8_t Channel;  //Current channel index
static TimerEvent_t ChannelCheckTimer;  //LoRaMac channel check timer
static void OnChannelCheckTimerEvent( void ); //Function executed on channel check timer event



//ADR 
static bool AdrCtrlOn = false;
static uint32_t AdrAckCounter = 0;

//入网
#define OVER_THE_AIR_ACTIVATION     0
uint32_t JoinAcceptDelay1 = 5000000;
uint32_t JoinAcceptDelay2 = 6000000;
bool IsLoRaMacNetworkJoined = false;
static uint16_t LoRaMacDevNonce;
TimerEvent_t JoinReqTimer;
void LoRaMacJoinReq(void);
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE          10000000  // 10 [s] value in us

//发送
#define LoRa_TxBuf_Size 128
static uint8_t LoRa_TxBuf[LoRa_TxBuf_Size];
static uint16_t LoRa_TxBuf_PktLen = 0;
uint32_t UpLinkCounter = 0;
static bool IsUpLinkCounterFixed = false;
#define LORAMAC_MFR_LEN             4    //Sets the length of the LoRaMAC footer field. Mainly indicates the MIC field length
uint8_t LoRaMacSendFrame( uint8_t fPort, void *fBuffer, uint16_t fBufferSize );
uint8_t LoRaMacSendFrameOnChannel( ChannelParams_t channel );

//延迟发送
uint64_t TxTimeOnAir = 0;        //Last transmission time on air
static bool DutyCycleOn; //Enables/Disables duty cycle management (Test only)
static uint8_t MaxDCycle = 0;   //Maximum duty cycle，\remark Possibility to shutdown the device. Everything else not implemented.
static uint16_t AggregatedDCycle;    //Agregated duty cycle management
static uint64_t AggregatedLastTxDoneTime;
static uint64_t AggregatedTimeOff;
static TimerEvent_t TxDelayedTimer;     //LoRaMac duty cycle delayed Tx timer
static void OnTxDelayedTimerEvent( void ); //Function executed on duty cycle delayed Tx  timer event

//应答、重发
bool AckReceived;
static bool NodeAckRequested = false;
static uint8_t AckTimeoutRetries = 3;        //重发最大次数
static uint8_t AckTimeoutRetriesCounter = 1; //重发次数
static bool AckTimeoutRetry = false;         //Indicates if the AckTimeout timer has expired or not
static TimerEvent_t AckTimeoutTimer;         //Acknowledge timeout timer. Used for packet retransmissions.
#define ACK_TIMEOUT                 2000000
#define ACK_TIMEOUT_RND             1000000
static void OnAckTimeoutTimerEvent( void ); //Function executed on AckTimeout timer event


//接收
#define LoRa_RxBuf_Size 128
static uint8_t LoRa_RxBuf[LoRa_RxBuf_Size];
static uint32_t DownLinkCounter = 0;
static bool SrvAckRequested = false;
uint32_t ReceiveDelay1 = 1000000;   //单位：us
uint32_t ReceiveDelay2 = 2000000;
static uint8_t Rx1DrOffset = 0; //Datarate offset between uplink and downlink on first window


//接收窗口
#define MAX_RX_WINDOW               3000000   //Class A&B maximum receive window delay in ms，LoRaMac maximum time a reception window stays open
static TimerEvent_t RxWindowTimer1;     //LoRaMac reception windows timers
static uint32_t RxWindow1Delay;
static void OnRxWindow1TimerEvent( void ); //Function executed on first Rx window timer event
static TimerEvent_t RxWindowTimer2;
static uint32_t RxWindow2Delay;
static void OnRxWindow2TimerEvent( void ); //Function executed on second Rx window timer event



LoRaMacEventFlags_t LoRaMacEventFlags;  //LoRaMac notification event flags

enum LoRaMacState_e   //LoRaMac internal states
{
        MAC_IDLE          = 0x00000000,
        MAC_TX_RUNNING    = 0x00000001,
        MAC_RX            = 0x00000002,
        MAC_ACK_REQ       = 0x00000004,
        MAC_ACK_RETRY     = 0x00000008,
        MAC_CHANNEL_CHECK = 0x00000010,
};




uint8_t LoRaMacEventInfo_RxSnr;
uint32_t LoRaMacState = MAC_IDLE;       //LoRaMac internal state
static TimerEvent_t MacStateCheckTimer; //LoRaMac timer used to check the LoRaMacState (runs every second)
#define MAC_STATE_CHECK_TIMEOUT     700000 //Check the Mac layer state every MAC_STATE_CHECK_TIMEOUT
static void OnMacStateCheckTimerEvent( void );  //Function executed on Resend Frame timer event.


/*!
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define LORAMAC_MIC_BLOCK_B0_SIZE                   16
static uint8_t MicBlockB0[] = { 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t aBlock[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t sBlock[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static aes_context AesContext;  //AES computation context variable
/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           4150 // mV
#define BATTERY_MIN_LEVEL                           3200 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3100 // mV


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

        mem_Init( enabledChannels, 0, LORA_MAX_NB_CHANNELS );

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
								if( LoRaMac_Channel_UpLink[i].Frequency == 0 )
								{
										// Check if the channel is enabled
										continue;
								}
								if( ( ( LoRaMac_Channel_UpLink[i].DrRange.Fields.Min <= ChannelsDatarate ) &&
																( ChannelsDatarate <= LoRaMac_Channel_UpLink[i].DrRange.Fields.Max ) ) == false )
								{
												// Check if the current channel selection supports the given datarate
												continue;
								}
								if( Bands[LoRaMac_Channel_UpLink[i].Band].TimeOff > 0 )
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
								if( SX1276IsChannelFree( MODEM_LORA, LoRaMac_Channel_UpLink[channelNext].Frequency) == true )
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
                        LoRaMacSendFrameOnChannel( LoRaMac_Channel_UpLink[Channel] );
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





static TimerEvent_t TxNextPacketTimer; //Defines the application data transmission duty cycle
volatile static bool TxDone = false;
u32 MoteIDAddr;
static u32 Rxpacket_count = 0 ;


void LoRaMacInit(void)
{
	  SX1276_Init();
		DownLinkCounter = 0;
		LoRaMacState = MAC_IDLE;

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
    //TxNextPacket = true;
    //TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );//20160826
	 
		ChannelsTxPower = TX_POWER_14_DBM;
		ChannelsMask = 0xFF ;
	
		AdrCtrlOn = true;
		ChannelsDefaultDatarate = ChannelsDatarate = DR_0;	
#if( OVER_THE_AIR_ACTIVATION != 0 )				
		LoRaMacJoinReq();
#endif		
    while(IsLoRaMacNetworkJoined==0)
		{
			TimerTask();
		}
}


/*
PHYpayload=MHDR(1B)+JoinRequest(18)+MIC(4B)
MHDR=MType(3b)+RFU(3b)+Major(2b)
JoinRequest=AppEUI(8)+DevEUI(8)+DevNonce(2)
MIC：aes128_cmac(NewSKey,B0|MHDR|JoinRequest)，取结果的低4位
*/
void LoRaMacJoinReq(void)
{
		uint8_t MHDR=0x00; 
		uint32_t MIC = 0;
	
		if( LoRaMacSetNextChannel() == 0 )
		{
				NodeAckRequested = false;
				LoRa_TxBuf[0] = MHDR;
				RxWindow1Delay = JoinAcceptDelay1 - RADIO_WAKEUP_TIME;
				RxWindow2Delay = JoinAcceptDelay2 - RADIO_WAKEUP_TIME;
				mem_Copy( LoRa_TxBuf + 1, LoRaMacAppEui, 8 ); //LoRa_TxBuf[1~8]
				mem_Copy( LoRa_TxBuf + 9, LoRaMacDevEui, 8 ); //LoRa_TxBuf[9~16]
									
				LoRaMacDevNonce=SX1276Random();										
				LoRa_TxBuf[17] = LoRaMacDevNonce & 0xFF;
				LoRa_TxBuf[18] = ( LoRaMacDevNonce >> 8 ) & 0xFF;
			
			  AES_CMAC_Join(LoRaMacAppKey,LoRa_TxBuf,0x13,&MIC);
				LoRa_TxBuf[19] = MIC & 0xFF;
				LoRa_TxBuf[20] = ( MIC >> 8 ) & 0xFF;
				LoRa_TxBuf[21] = ( MIC >> 16 ) & 0xFF;
				LoRa_TxBuf[22] = ( MIC >> 24 ) & 0xFF;
        LoRa_TxBuf_PktLen=23;
				AckReceived = false;
				if( ChannelsDatarate == DR_6 )// High speed LoRa channel
				{      					
						SX1276_TxConfig( MODEM_LORA, LoRaMac_Channel_UpLink[Channel].Frequency,TxPowers[ChannelsTxPower], 0, 1, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6,&TxTimeOnAir,LoRa_TxBuf_PktLen );
				}
				else// Normal LoRa channel
				{      
						SX1276_TxConfig( MODEM_LORA, LoRaMac_Channel_UpLink[Channel].Frequency,TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6,&TxTimeOnAir,LoRa_TxBuf_PktLen );
				}
				if( MaxDCycle == 255 )
						return ;
				if( MaxDCycle == 0 )
						AggregatedTimeOff = 0;
				LoRaMacState |= MAC_TX_RUNNING;
				TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );						
				TimerEvent_Start( &MacStateCheckTimer );  //Starts the MAC layer status check timer

				if( MAX( Bands[LoRaMac_Channel_UpLink[Channel].Band].TimeOff, AggregatedTimeOff ) > ( TIM2_Time_Present() ))
				{
						TimerSetValue( &TxDelayedTimer, MAX( Bands[LoRaMac_Channel_UpLink[Channel].Band].TimeOff, AggregatedTimeOff ) );
						TimerEvent_Start( &TxDelayedTimer );
				}
				else
				{
					SX1276_Send( LoRa_TxBuf, 23);
				}							
		}
}




uint8_t LoRaMacSendFrameOnChannel( ChannelParams_t channel )
{
        u8 debug_buf[32];
        if( ChannelsDatarate == DR_6 )// High speed LoRa channel
        {       
						SX1276_TxConfig( MODEM_LORA, channel.Frequency, TxPowers[ChannelsTxPower], 0, 1, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 ,&TxTimeOnAir,LoRa_TxBuf_PktLen );
        }
        else// Normal LoRa channel
        { 
						SX1276_TxConfig( MODEM_LORA, channel.Frequency ,TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 ,&TxTimeOnAir, LoRa_TxBuf_PktLen);
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
          SX1276_Send( LoRa_TxBuf, LoRa_TxBuf_PktLen );				
        }
        return 0;
}

void OnTxDelayedTimerEvent( void )
{
    SX1276_Send( LoRa_TxBuf, LoRa_TxBuf_PktLen );
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
	Bands[LoRaMac_Channel_UpLink[Channel].Band].LastTxDoneTime = curTime;
	if( DutyCycleOn == true )
	{
			Bands[LoRaMac_Channel_UpLink[Channel].Band].TimeOff = TxTimeOnAir * Bands[LoRaMac_Channel_UpLink[Channel].Band].DCycle - TxTimeOnAir;
	}
	else
	{
			Bands[LoRaMac_Channel_UpLink[Channel].Band].TimeOff = 0;
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


/*
PHYpayload = MHDR[MType(3b)+RFU(3b)+Major(2b)] + JoinAccept/MACPayload + MIC(4B)
JoinAccept=APPNonce(3)+NetID(3)+DevAddr(4)+DLSetting(1)+RxDelay(1)+CFList(16,Optiona)
MACPayload=[DevAddr(4)+FCtr(1)+FCnt(2)+FOpts(0~15B)]+FPort(1)+FRMPayload
MIC：aes128_cmac(NewSKey,B0|MHDR|MACPayload)，取结果的低4位
--------------------------------
FCtr=ADR+RFU+ACK+FPending+FOptsLen(4b)
FOpts包含未加密的数据，FRMPayload包含加密的数据(FPort=0时，FRMPayload只包含MAC命令)
本函数现将FRMPayload解密，然后和FOpts合并，一起处理
*/
void OnRadioRxDone(uint8_t *PHYPayload, uint16_t size, int8_t rssi, int8_t snr )
{
	uint8_t MHDR = PHYPayload[0];
	uint8_t MType = MHDR>>5;
  uint32_t DevAddr = 0;
	
	uint8_t FCtr;
	uint8_t ACK;
	uint8_t FOptsLen;	
	
  uint16_t FCnt = 0;
		
  uint8_t FPort = 0xFF;
  uint8_t FRMPayload_Len = 0; //FRMPayload长度

  uint32_t MIC = 0;    //mic接收值	
  uint32_t MICc = 0;   //mic计算值	
  bool isMicOk = false;

	uint8_t FrameIndex = 0;
	uint8_t FrameLen = 0;	
	
  int32_t sequence = 0;

  MulticastParams_t *curMulticastParams = NULL;
  uint8_t *nwkSKey = LoRaMacNwkSKey;
  uint8_t *appSKey = LoRaMacAppSKey;
  uint32_t downLinkCounter = 0;


  SX1276SetSleep( );
  TimerEvent_Stop( &RxWindowTimer2 );

  switch( MType )
  {
		case 0x01:     //FRAME_TYPE_JOIN_ACCEPT
			if( IsLoRaMacNetworkJoined == true )
					break;

      Decrypt_Join(LoRaMacAppKey,PHYPayload+1,LoRa_RxBuf+1,size-1); //Decrypt结果放在LoRaMacRxPayload中
			LoRa_RxBuf[0] = MHDR;
			AES_CMAC_Join(LoRaMacAppKey,LoRa_RxBuf, (size - LORAMAC_MFR_LEN) & 0xFF,&MICc);
			MIC = (LoRa_RxBuf[size-LORAMAC_MFR_LEN+3]<<24)|(LoRa_RxBuf[size-LORAMAC_MFR_LEN+2]<<16)|(LoRa_RxBuf[size-LORAMAC_MFR_LEN+1]<<8)|(LoRa_RxBuf[size-LORAMAC_MFR_LEN]);
			if( MIC == MICc )
			{
					LoRaMacEventFlags.Bits.Rx = 1;
					LoRaMacEventInfo_RxSnr = snr;

					//LoRaMacJoinComputeSKeys( LoRaMacAppKey, LoRa_RxBuf + 1, LoRaMacDevNonce, LoRaMacNwkSKey, LoRaMacAppSKey );
					uint8_t nonce[16];
					uint8_t *pDevNonce = ( uint8_t * )&LoRaMacDevNonce;
					
					mem_Init( AesContext.ksch, '\0', 240 );
					aes_set_key( LoRaMacAppKey, 16, &AesContext );

					mem_Init( nonce, 0, sizeof( nonce ) );
					nonce[0] = 0x01;
					mem_Copy( nonce + 1, LoRa_RxBuf + 1, 6 );
					mem_Copy( nonce + 7, pDevNonce, 2 );
					aes_encrypt( nonce, LoRaMacNwkSKey, &AesContext );

					mem_Init( nonce, 0, sizeof( nonce ) );
					nonce[0] = 0x02;
					mem_Copy( nonce + 1, LoRa_RxBuf + 1, 6 );
					mem_Copy( nonce + 7, pDevNonce, 2 );
					aes_encrypt( nonce, appSKey, &AesContext );

					LoRaMacNetID = LoRa_RxBuf[4];
					LoRaMacNetID |= ( LoRa_RxBuf[5] << 8 );
					LoRaMacNetID |= ( LoRa_RxBuf[6] << 16 );
					LoRaMacDevAddr = LoRa_RxBuf[7];
					LoRaMacDevAddr |= ( LoRa_RxBuf[8] << 8 );
					LoRaMacDevAddr |= ( LoRa_RxBuf[9] << 16 );
					LoRaMacDevAddr |= ( LoRa_RxBuf[10] << 24 );

					Rx1DrOffset = ( LoRa_RxBuf[11] >> 4 ) & 0x07; // DLSettings
					Rx2Channel.Datarate = LoRa_RxBuf[11] & 0x0F;

					ReceiveDelay1 = ( LoRa_RxBuf[12] & 0x0F );
					if( ReceiveDelay1 == 0 )
							ReceiveDelay1 = 1;
					ReceiveDelay1 *= 1e6;
					ReceiveDelay2 = ReceiveDelay1 + 1e6;
					
					LoRaMacEventFlags.Bits.JoinAccept = 1;
					ChannelsDatarate = ChannelsDefaultDatarate;
					IsLoRaMacNetworkJoined = true;
			}
			LoRaMacEventFlags.Bits.Tx = 1;
			break;
  case 0x05:  //FRAME_TYPE_DATA_CONFIRMED_DOWN:
  case 0x03:  //FRAME_TYPE_DATA_UNCONFIRMED_DOWN  	  
  {
			DevAddr = (PHYPayload[4]<<24)|(PHYPayload[3]<<16)|(PHYPayload[2]<<8)|PHYPayload[1];//地址校验
			if( DevAddr != LoRaMacDevAddr ) //地址不匹配
			{
					curMulticastParams = MulticastChannels;
					while( curMulticastParams != NULL )
					{
							if( DevAddr == curMulticastParams->Address ) //地址不匹配，但是是多地址
							{
									LoRaMacEventFlags.Bits.Multicast = 1;
									nwkSKey = curMulticastParams->NwkSKey;
									appSKey = curMulticastParams->AppSKey;
									downLinkCounter = curMulticastParams->DownLinkCounter;
									break;
							}
							curMulticastParams = curMulticastParams->Next;
					}
					if( LoRaMacEventFlags.Bits.Multicast == 0 )      //地址不匹配，且不是多地址，直接退出
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

			FCtr = PHYPayload[5];
			ACK=(FCtr&0x20)>>5;	
			FOptsLen=FCtr&0x0F;		
			
			FCnt |= PHYPayload[6];
			FCnt |= PHYPayload[7] << 8;
			
			//MIC校验
			MIC=(PHYPayload[size-LORAMAC_MFR_LEN+3]<<24)|(PHYPayload[size-LORAMAC_MFR_LEN+2]<<16)|(PHYPayload[size-LORAMAC_MFR_LEN+1]<<8)|PHYPayload[size -LORAMAC_MFR_LEN];

			sequence = ( int32_t )FCnt - ( int32_t )( downLinkCounter & 0xFFFF );
			if( sequence < 0 )  // sequence reset or roll over happened
			{
					downLinkCounter = ( downLinkCounter & 0xFFFF0000 ) | ( FCnt + ( uint32_t )0x10000 );
					MicBlockB0[5] = 1;   //DOWN_LINK  
					MicBlockB0[6] = ( DevAddr ) & 0xFF;
					MicBlockB0[7] = ( DevAddr >> 8 ) & 0xFF;
					MicBlockB0[8] = ( DevAddr >> 16 ) & 0xFF;
					MicBlockB0[9] = ( DevAddr >> 24 ) & 0xFF;
					MicBlockB0[10] = ( downLinkCounter ) & 0xFF;
					MicBlockB0[11] = ( downLinkCounter >> 8 ) & 0xFF;
					MicBlockB0[12] = ( downLinkCounter >> 16 ) & 0xFF;
					MicBlockB0[13] = ( downLinkCounter >> 24 ) & 0xFF;
					MicBlockB0[15] = (size - LORAMAC_MFR_LEN) & 0xFF;
			    AES_CMAC2(nwkSKey,MicBlockB0,LORAMAC_MIC_BLOCK_B0_SIZE,PHYPayload,(size-LORAMAC_MFR_LEN)&0xFF,&MICc);						
					if( MIC == MICc )
							isMicOk = true;
					else
					{
							isMicOk = false;
							if( LoRaMacEventFlags.Bits.Multicast == 1 )// sequence reset
									curMulticastParams->DownLinkCounter = downLinkCounter = FCnt;
							else
									DownLinkCounter = downLinkCounter = FCnt;
							MicBlockB0[5] = 1;   //DOWN_LINK  
							MicBlockB0[6] = ( DevAddr ) & 0xFF;
							MicBlockB0[7] = ( DevAddr >> 8 ) & 0xFF;
							MicBlockB0[8] = ( DevAddr >> 16 ) & 0xFF;
							MicBlockB0[9] = ( DevAddr >> 24 ) & 0xFF;
							MicBlockB0[10] = ( downLinkCounter ) & 0xFF;
							MicBlockB0[11] = ( downLinkCounter >> 8 ) & 0xFF;
							MicBlockB0[12] = ( downLinkCounter >> 16 ) & 0xFF;
							MicBlockB0[13] = ( downLinkCounter >> 24 ) & 0xFF;
							MicBlockB0[15] = (size - LORAMAC_MFR_LEN) & 0xFF;
				      AES_CMAC2(nwkSKey,MicBlockB0,LORAMAC_MIC_BLOCK_B0_SIZE,PHYPayload,(size-LORAMAC_MFR_LEN)&0xFF,&MICc);				
					}
			}
			else
			{
					downLinkCounter = ( downLinkCounter & 0xFFFF0000 ) | FCnt;
					MicBlockB0[5] = 1;   //DOWN_LINK  
					MicBlockB0[6] = ( DevAddr ) & 0xFF;
					MicBlockB0[7] = ( DevAddr >> 8 ) & 0xFF;
					MicBlockB0[8] = ( DevAddr >> 16 ) & 0xFF;
					MicBlockB0[9] = ( DevAddr >> 24 ) & 0xFF;
					MicBlockB0[10] = ( downLinkCounter ) & 0xFF;
					MicBlockB0[11] = ( downLinkCounter >> 8 ) & 0xFF;
					MicBlockB0[12] = ( downLinkCounter >> 16 ) & 0xFF;
					MicBlockB0[13] = ( downLinkCounter >> 24 ) & 0xFF;
					MicBlockB0[15] = (size - LORAMAC_MFR_LEN) & 0xFF;
				  AES_CMAC2(nwkSKey,MicBlockB0,LORAMAC_MIC_BLOCK_B0_SIZE,PHYPayload,(size-LORAMAC_MFR_LEN)&0xFF,&MICc);
			}

			if((isMicOk ==true)||(MIC == MICc))   //mic校验正确，执行接收程序
			{
					LoRaMacEventFlags.Bits.Rx = 1;
					LoRaMacEventInfo_RxSnr = snr;
					AdrAckCounter = 0;
					if( LoRaMacEventFlags.Bits.Multicast == 1 )
					{
							curMulticastParams->DownLinkCounter = downLinkCounter;
					}
					else
					{
							DownLinkCounter = downLinkCounter;
					}

					if( MType == 0x05 ) //FRAME_TYPE_DATA_CONFIRMED_DOWN
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

					if(FOptsLen > 0 )          //FOpts存在, FOpts包含未加密的MAC命令
					{
						mem_Copy(LoRa_RxBuf,PHYPayload+8,FOptsLen);
						FrameLen = FOptsLen;
					}
					if((size-8-FOptsLen-4)>0)  //FPort存在, FRMPayload包含加密的MAC命令，先解密再存放到LoRa_RxBuf****
					{
							uint16_t i;
							uint16_t ctr = 1;
							FPort = PHYPayload[8+FOptsLen];
							FRMPayload_Len =size-8-FOptsLen-1-4;
							aBlock[5] = 1;   //DOWN_LINK  
							aBlock[6] = ( DevAddr ) & 0xFF;
							aBlock[7] = ( DevAddr >> 8 ) & 0xFF;
							aBlock[8] = ( DevAddr >> 16 ) & 0xFF;
							aBlock[9] = ( DevAddr >> 24 ) & 0xFF;
							aBlock[10] = ( downLinkCounter ) & 0xFF;
							aBlock[11] = ( downLinkCounter >> 8 ) & 0xFF;
							aBlock[12] = ( downLinkCounter >> 16 ) & 0xFF;
							aBlock[13] = ( downLinkCounter >> 24 ) & 0xFF;						
							mem_Init( AesContext.ksch, '\0', 240 );
						  if( FPort == 0 )
							  aes_set_key( nwkSKey, 16, &AesContext );
							else
								aes_set_key( appSKey, 16, &AesContext );
							while( FRMPayload_Len >= 16 ) //FRMPayload分组解密，每组16B，
							{
									aBlock[15] = ( ( ctr ) & 0xFF );
									ctr++;
									aes_encrypt( aBlock, sBlock, &AesContext );
									for( i = 0; i < 16; i++ )  //FRMPayload-decode，结果存在LoRaMacRxPayload
											LoRa_RxBuf[FrameLen+i]=*(PHYPayload+8+FOptsLen+1+FrameLen+i)^sBlock[i];
									FRMPayload_Len -= 16;
									FrameLen += 16;
							}
							if( FRMPayload_Len > 0 )    //最后一组，不满16B
							{
									aBlock[15] = ( ( ctr ) & 0xFF );
									aes_encrypt( aBlock, sBlock, &AesContext );
									for( i = 0; i < FRMPayload_Len; i++ )
											LoRa_RxBuf[FrameLen++]=(*(PHYPayload+8+FOptsLen+1+FrameLen))^sBlock[i];
							}   
						}											
						while( FrameIndex < FrameLen )  //MAC命令可能不止一个，使用while循环逐个处理
						{
								switch( LoRa_RxBuf[FrameIndex++] )
								{
									case 0x02:    //SRV_MAC_LINK_CHECK_ANS
											LoRaMacEventFlags.Bits.LinkCheck = 1;
									break;
									case 0x03:   //SRV_MAC_LINK_ADR_REQ
									{
											uint8_t i;
											uint8_t status = 0x07;
											uint16_t chMask = 0;
											int8_t txPower = 0;
											int8_t datarate = 0;
											uint8_t nbRep = 0;
											uint8_t chMaskCntl = 0;
											datarate = LoRa_RxBuf[FrameIndex++];
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
											chMask = LoRa_RxBuf[FrameIndex++];
											chMask |= LoRa_RxBuf[FrameIndex++] << 8;
											nbRep = LoRa_RxBuf[FrameIndex++];
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
																		if( LoRaMac_Channel_UpLink[i].Frequency != 0 )
																						chMask |= 1 << i;
														}
														else
														{
																		if(((chMask&(1<<i))!= 0)&&(LoRaMac_Channel_UpLink[i].Frequency == 0 ) )
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
									case 0x04:          //SRV_MAC_DUTY_CYCLE_REQ
											MaxDCycle = LoRa_RxBuf[FrameIndex++];
											AggregatedDCycle = 1 << MaxDCycle;
											AddMacCommand( MOTE_MAC_DUTY_CYCLE_ANS, 0, 0 );
									break;
									case 0x05:  //SRV_MAC_RX_PARAM_SETUP_REQ
									{
											uint8_t status = 0x07;
											int8_t datarate = 0;
											int8_t drOffset = 0;
											uint32_t freq = 0;

											drOffset = LoRa_RxBuf[FrameIndex++];
											datarate = drOffset & 0x0F;
											drOffset = ( drOffset >> 4 ) & 0x0F;

											freq = LoRa_RxBuf[FrameIndex++];
											freq |= LoRa_RxBuf[FrameIndex++] << 8;
											freq |= LoRa_RxBuf[FrameIndex++] << 16;
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
									case 0x06:   //SRV_MAC_DEV_STATUS_REQ
											AddMacCommand( MOTE_MAC_DEV_STATUS_ANS, BoardMeasureBatterieLevel( ), LoRaMacEventInfo_RxSnr );
											break;
									case 0x07:  //SRV_MAC_NEW_CHANNEL_REQ
									{
											uint8_t status = 0x03;
											int8_t channelIndex = 0;
											ChannelParams_t chParam;
											channelIndex = LoRa_RxBuf[FrameIndex++];
											chParam.Frequency = LoRa_RxBuf[FrameIndex++];
											chParam.Frequency |= LoRa_RxBuf[FrameIndex++] << 8;
											chParam.Frequency |= LoRa_RxBuf[FrameIndex++] << 16;
											chParam.Frequency *= 100;
											chParam.DrRange.Value = LoRa_RxBuf[FrameIndex++];
											if((channelIndex<3)||( channelIndex>LORA_MAX_NB_CHANNELS))
															status &= 0xFE; // Channel frequency KO
											if((chParam.DrRange.Fields.Min > chParam.DrRange.Fields.Max )||(((LORAMAC_MIN_DATARATE <=chParam.DrRange.Fields.Min)&&(chParam.DrRange.Fields.Min <= LORAMAC_MAX_DATARATE))==false)||(((LORAMAC_MIN_DATARATE <= chParam.DrRange.Fields.Max)&&(chParam.DrRange.Fields.Max <= LORAMAC_MAX_DATARATE))==false))
															status &= 0xFD; // Datarate range KO
											if( ( status & 0x03 ) == 0x03 )
											{
													chParam.Band = 0;
													LoRaMac_Channel_UpLink[channelIndex] = chParam;
													// Activate the newly created channel
													ChannelsMask |= 1 << channelIndex;
													if( LoRaMac_Channel_UpLink[channelIndex].Frequency == 0 )
																	ChannelsMask &= ~( 1 << channelIndex );			
											}
											AddMacCommand( MOTE_MAC_NEW_CHANNEL_ANS, status, 0 );
									}
									break;
									case 0x08:   //SRV_MAC_RX_TIMING_SETUP_REQ
									{
											uint8_t delay = LoRa_RxBuf[FrameIndex++] & 0x0F;
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
              LoRaMacEventFlags.Bits.Tx = 1;						
					}	
					else  //mic校验错误，不执行上面的接收程序，
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

        //LoRaMacRxWindowSetup( LoRaMac_Channel_UpLink[Channel].Frequency, datarate, bandwidth, symbTimeout, false );{
        if( SX1276_State == RF_IDLE )
        {
						SX1276_RxConfig( MODEM_LORA,LoRaMac_Channel_DownLink[Channel].Frequency, bandwidth, Datarates[datarate], 1, 0, 8, symbTimeout, false, 0, false, 0, 0, true, false,MAX_RX_WINDOW );
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
													LoRaMacSendFrameOnChannel( LoRaMac_Channel_UpLink[Channel] );
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
									LoRaMacSendFrameOnChannel( LoRaMac_Channel_UpLink[Channel] );
							}
					}    
					else                                                //重发次数达到上限
					{
									ChannelsMask = ChannelsMask | 0x07;// Re-enable default channels LC1, LC2, LC3
									LoRaMacState &= ~MAC_TX_RUNNING;
									AckReceived = false;                      //本次发送的结果：未收到ACK 
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

/*
PHYpayload=MHDR(1B)+MACPayload+MIC(4B)
MHDR=MType(3b)+RFU(3b)+Major(2b)
MACPayload=[DevAddr(4)+FCtr(1)+FCnt(2)+FOpts(0~15B)]+FPort(1)+FRMPayload
JoinAccept=APPNonce(3)+NetID(3)+DevAddr(4)+DLSetting(1)+RxDelay(1)+CFList(16,Optiona)
MIC：aes128_cmac(NewSKey,B0|MHDR|MACPayload)，取结果的低4位
--------------------------------
FCtr=ADR+RFU+ACK+FPending+FOptsLen(4b)
*/
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
	LoRa_TxBuf_PktLen = 0;	
	uint64_t curTime = TIM2_Time_Present();

	uint8_t bufferIndex = 0;
	uint16_t ctr = 1;	

	RxWindow1Delay = ReceiveDelay1 - RADIO_WAKEUP_TIME;
	RxWindow2Delay = ReceiveDelay2 - RADIO_WAKEUP_TIME;

	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = MHDR;		
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( LoRaMacDevAddr ) & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( LoRaMacDevAddr >> 24 ) & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = FCtr;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = FCnt & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( FCnt >> 8 ) & 0xFF;
	if( FOpts != NULL )
	{
		for(uint8_t i=0; i<FOptsLen; i++ )
			LoRa_TxBuf[LoRa_TxBuf_PktLen++] = FOpts[i];
	}	
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = FPort;

	//LoRaMacPayloadEncrypt
	mem_Init( AesContext.ksch, '\0', 240 );
	aes_set_key( LoRaMacAppSKey, 16, &AesContext );
	aBlock[5] = 0;  //UP_LINK
	aBlock[6] = ( LoRaMacDevAddr ) & 0xFF;
	aBlock[7] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
	aBlock[8] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
	aBlock[9] = ( LoRaMacDevAddr >> 24 ) & 0xFF;
	aBlock[10] = ( UpLinkCounter ) & 0xFF;
	aBlock[11] = ( UpLinkCounter >> 8 ) & 0xFF;
	aBlock[12] = ( UpLinkCounter >> 16 ) & 0xFF;
	aBlock[13] = ( UpLinkCounter >> 24 ) & 0xFF;
	while( fBufferSize >= 16 )   //fBuffer分组加密
	{
			aBlock[15] = ( ( ctr ) & 0xFF );
			ctr++;
			aes_encrypt( aBlock, sBlock, &AesContext );
			for( uint16_t i = 0; i < 16; i++ )
					LoRa_TxBuf[LoRa_TxBuf_PktLen++] = fBuffer[bufferIndex + i] ^ sBlock[i];
			fBufferSize -= 16;
			bufferIndex += 16;
	}

	if( fBufferSize > 0 )
	{
			aBlock[15] = ( ( ctr ) & 0xFF );
			aes_encrypt( aBlock, sBlock, &AesContext );
			for( uint16_t i = 0; i < fBufferSize; i++ )
					LoRa_TxBuf[LoRa_TxBuf_PktLen++] = fBuffer[bufferIndex + i] ^ sBlock[i];
	}


	//LoRaMacComputeMic( LoRa_TxBuf, LoRa_TxBuf_PktLen, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &MIC );
	MicBlockB0[5] = 0; //UP_LINK
	MicBlockB0[6] = ( LoRaMacDevAddr ) & 0xFF;
	MicBlockB0[7] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
	MicBlockB0[8] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
	MicBlockB0[9] = ( LoRaMacDevAddr >> 24 ) & 0xFF;
	MicBlockB0[10] = ( UpLinkCounter ) & 0xFF;
	MicBlockB0[11] = ( UpLinkCounter >> 8 ) & 0xFF;
	MicBlockB0[12] = ( UpLinkCounter >> 16 ) & 0xFF;
	MicBlockB0[13] = ( UpLinkCounter >> 24 ) & 0xFF;
	MicBlockB0[15] = LoRa_TxBuf_PktLen & 0xFF;
  AES_CMAC2(LoRaMacNwkSKey,MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE,LoRa_TxBuf, LoRa_TxBuf_PktLen & 0xFF ,&MIC);
				
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = MIC & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( MIC >> 8 ) & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( MIC >> 16 ) & 0xFF;
	LoRa_TxBuf[LoRa_TxBuf_PktLen++] = ( MIC >> 24 ) & 0xFF;	



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
			channel=LoRaMac_Channel_UpLink[Channel];	
			
			AckReceived = false;
			
			SX1276_TxConfig( MODEM_LORA, channel.Frequency,TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 , &TxTimeOnAir, LoRa_TxBuf_PktLen );// Normal LoRa channel
			LoRaMacState |= MAC_TX_RUNNING;
		  TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT );	
		  //TimerEvent_Start( &MacStateCheckTimer );// Starts the MAC layer status check timer

			SX1276_Send(LoRa_TxBuf, LoRa_TxBuf_PktLen);	
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



