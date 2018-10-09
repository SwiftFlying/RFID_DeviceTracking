#include "app.h"


#define TagCount 8
#define TAGStopTime 4

typedef struct TAG_t{
	uint8_t EPC1;
	uint8_t EPC2;	
	uint16_t StopTime;
	uint8_t state;
}tag_t;

#define tags_count 3
tag_t tags[tags_count]=
{
	{0x93,0x33,0,0},
	{0xe0,0x9d,0,0},
	{0x09,0x42,0,0},
};

tag_t *tag_head;
tag_t TAG[TagCount];
uint8_t EPC[1000];
uint8_t top;
uint8_t rear;

uint16_t rf_tick=0;
uint16_t StopTime_tick=0;
uint8_t Multi_Inventory[10]={0xBB,0x00,0x27,0x00,0x03,0x22,0xFF,0xFF,0x4A,0x7E};
uint8_t Stop[7]={0xBB,0x00,0x28,0x00,0x00,0x28,0x7E};
uint8_t Single_Inventory[7]={0xBB,0x00,0x22,0x00,0x00,0x22,0x7E};

uint8_t modem_frame[10]={0xff,0xff,0x41,0x01,0x00,0x02,0x00,0x03,0x00,0xaa};

void RFID_Init(void)
{
	DelayMs(1000);  //µÈ´ýRF100Æô¶¯Íê±Ï
	UART_Send(Multi_Inventory ,10 );	
}


void RFID_Task(void)
{
		if(StopTime_tick>=1000)
		{
			  StopTime_tick=0;
				for(uint8_t i=0;i<tags_count;i++)
			  {
						tags[i].StopTime++;
						if((tags[i].state==1) && (tags[i].StopTime>=TAGStopTime))		
							tags[i].state=0;				
				}
		}

		while(UART_RxBuf_Rear!=UART_RxBuf_Top)
		{	 
				uint8_t a,b;
				a=UART_RxBuf[UART_RxBuf_Rear];
				UART_RxBuf_Rear=(UART_RxBuf_Rear+1)%UART_RxBuf_Size;
				b=UART_RxBuf[UART_RxBuf_Rear];
				UART_RxBuf_Rear=(UART_RxBuf_Rear+1)%UART_RxBuf_Size;
				for(uint8_t j=0;j<tags_count;j++)
			  {
						if(tags[j].EPC1==a && tags[j].EPC2==b)
						{
							tags[j].StopTime=0;
							if(tags[j].state==0)
								tags[j].state=1; 
						}				
				}		 
		}
		
		if(rf_tick>=5000)
		{
				rf_tick=0;
				UART_Send(Multi_Inventory ,10 );			
				modem_frame[4]=tags[0].state;  //00 42
				modem_frame[6]=tags[1].state;  //06 55
				modem_frame[8]=tags[2].state;  //07 39 
				LoRaMac_Send(modem_frame,10,0);			
		} 	  	 
}



