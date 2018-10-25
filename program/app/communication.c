/*
(C)2016 eolane

Description: communicate with server

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Alan.zhu
*/

#include "bsp_Config.h"


#define SOF 0x55



#define CMD_ACK_MASK 0x80

#define LIGHT_OPERATE 0x01
#define LIGHT_STATUS 0x02

#define SET_LIGHT_VALUE 0x03
#define READ_LIGHT_VALUE 0x04

#define SetTxHead() {transmitBuf[0] = SOF;}
#define SetTxPayLoadLen(len) {transmitBuf[1] = len;}
#define SetTxCmd(cmd) {transmitBuf[2] = cmd;}
#define SetTxData(index,data) {transmitBuf[3+index] = data;}



uint8_t transmitBuf[70];
uint8_t transmitStatus = 0;
volatile uint8_t pwm_scale=0;//high level time = 100us*pwm_scale	


void CmdProcessNormal(uint8_t cmd,uint8_t* pDataIn,uint8_t dataLen)
{
	static uint8_t lightStatus=0;	
	uint8_t dataIn;	
	uint8_t lightValue;
	
	switch(cmd)
	{
		case LIGHT_OPERATE:
			dataIn=*pDataIn;
			if(dataIn==0x00)
			{
				lightStatus = 0;
				//HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);					
			}
			else if(dataIn==0x01)	
			{
				lightStatus = 1;
				//HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);	
				SetPwmScale(10);
			}	
			SetTxPayLoadLen(1);
		break;
		
		case LIGHT_STATUS:
			SetTxData(0,lightStatus);
			SetTxPayLoadLen(2);				
		break;
		
		case SET_LIGHT_VALUE:
			lightValue = *pDataIn;
		  SetPwmScale(lightValue);		
		  SetTxPayLoadLen(1);
		break;
		
		case READ_LIGHT_VALUE:
			SetTxData(0,GetPwmScale());
			SetTxPayLoadLen(2);	
		break;
		
		default:
		break;
	}
}

uint8_t CmdProcess(uint8_t *pFrameIn)
{
	uint8_t sof,cmd;
	uint8_t payLoadLen;//include command length and data length
	uint8_t dataLen;
	uint8_t *pBuf;
	uint8_t *pData;
	uint8_t state;
	
	
	pBuf = pFrameIn;
	sof = *pBuf++;
	payLoadLen = *pBuf++;
	dataLen = payLoadLen -1;
	cmd = *pBuf++;
	pData = pBuf;
	
	
	if(sof!=SOF)
	{
		state = COMMUNICATION_SOF_ERROR;
		return state;
	}
	else
	{
		SetTxHead();
		SetTxCmd((cmd|CMD_ACK_MASK));
		CmdProcessNormal(cmd,pData,dataLen);
		state = COMMUNICATION_SUCCESS;
		return state;
	}	
}


	
