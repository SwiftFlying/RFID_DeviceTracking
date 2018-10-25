#include "../com.h"

#define SPI_NSS_LOW()            HAL_GPIO_WritePin(SPIx_GPIO_PORT, SPIx_NSS_PIN, GPIO_PIN_RESET)
#define SPI_NSS_HIGH()           HAL_GPIO_WritePin(SPIx_GPIO_PORT, SPIx_NSS_PIN, GPIO_PIN_SET)

//Writes multiple radio registers starting at address
void SPI_Send( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t write_addr = addr | 0x80;  //8位地址写入，第一位是1表示是写操作
	SPI_NSS_LOW();
	HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &write_addr, 1, SPIx_TIMEOUT_MAX);
	for(uint8_t i=0; i<size ; i++ )
			HAL_SPI_Transmit(&SpiHandle, (uint8_t*)(buffer+i), 1, SPIx_TIMEOUT_MAX);
	SPI_NSS_HIGH();
}

//Reads multiple radio registers starting at address
void SPI_Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t read_addr = addr & 0x7F;
	SPI_NSS_LOW();
  HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &read_addr, 1, SPIx_TIMEOUT_MAX);
	for(uint8_t i=0; i<size ; i++ )
	{	
		uint32_t readvalue = 0;
		uint32_t writevalue = 0xFFFFFFFF;
		HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SPIx_TIMEOUT_MAX);	
    buffer[i] = readvalue;
  }
	SPI_NSS_HIGH();
}


