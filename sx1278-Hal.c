/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1278-Hal.c
 * \brief      SX1278 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Marc0 Xu on Jan 07 2018
 */
//#include <stdint.h>
//#include <stdbool.h> 

#include "platform.h"
#include "RadioConfig.h"
#if defined( USE_SX1278_RADIO )

#include "sx1278-Hal.h"

/*!
 * SX1278 RESET I/O definitions
 */


/*!
 * SX1278 SPI NSS I/O definitions
 */
  

#define SPI_NSS_LOW()            HAL_GPIO_WritePin(NSS_IOPORT , NSS_PIN, GPIO_PIN_RESET)
#define SPI_NSS_HIGH()           HAL_GPIO_WritePin(NSS_IOPORT , NSS_PIN, GPIO_PIN_SET)

#define DUMMY_BYTE   0



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure system clock to run at 16MHz
  * @param  None
  * @retval None
  */
void BoardInit( void )
{
  HAL_GPIO_WritePin(RST_IOPORT,RST_PIN,GPIO_PIN_RESET);
  osDelay(10);
  HAL_GPIO_WritePin(RST_IOPORT,RST_PIN,GPIO_PIN_SET);
  osDelay(10);

}

void SX1278InitIo( void )
{
  
  //EXTI_SetPinSensitivity(EXTI_Pin_5, EXTI_Trigger_Rising);
}

/**********************************************************
* @brief  Sends a byte through the SPI interface and 
					return the byte received from the SPI bus.
* @param  byte: byte to send.
* @retval The value of the received byte.
**********************************************************/
static uint8_t SPI1_SendByte(uint8_t byte)
{
  uint8_t Readbyte;
  HAL_SPI_TransmitReceive(&SX1278SPI,&byte,&Readbyte,1,100);
  return Readbyte;
}
/**********************************************************
**Name:     SPIWrite
**Function: SPI Write CMD
**Input:    uint8_t address & uint8_t data
**Output:   None
**********************************************************/
void SPIWrite(uint8_t adr, uint8_t WrPara)  
{	
  SPI_NSS_LOW();						
  SPI1_SendByte(adr|0x80);		 
  SPI1_SendByte(WrPara);                
  SPI_NSS_HIGH();

}
/**********************************************************
**Name:     SPIRead
**Function: SPI Read CMD
**Input:    adr -> address for read
**Output:   None
**********************************************************/
uint8_t SPIRead(uint8_t adr)
{
  uint8_t tmp; 
  SPI_NSS_LOW();
  SPI1_SendByte(adr);                          
  tmp = SPI1_SendByte(DUMMY_BYTE);
  SPI_NSS_HIGH();  
  return(tmp);
}
/**********************************************************
**Name:     SX1278ReadBuffer
**Function: SPI burst read mode
**Input:    adr-----address for read
**          ptr-----data buffer point for read
**          length--how many bytes for read
**Output:   None
**********************************************************/

void SX1278ReadBuffer(uint8_t adr, uint8_t *ptr, uint8_t length)
{
  uint8_t i;
  
  SPI_NSS_LOW();
  SPI1_SendByte(adr); 
  for(i=0;i<length;i++)
  ptr[i] = SPI1_SendByte(DUMMY_BYTE);
  SPI_NSS_HIGH(); 
}
/**********************************************************
**Name:     SX1278WriteBuffer
**Function: SPI burst write mode
**Input:    adr-----address for write
**          ptr-----data buffer point for write
**          length--how many bytes for write
**Output:   none
**********************************************************/      
void SX1278WriteBuffer(uint8_t adr, uint8_t *ptr, uint8_t length)
{ 
  uint8_t i;

  SPI_NSS_LOW();       
  SPI1_SendByte(adr|0x80);
  for(i=0;i<length;i++)
  SPI1_SendByte(ptr[i]);
  SPI_NSS_HIGH(); 
}

void SX1278Write( uint8_t addr, uint8_t data )
{
  SX1278WriteBuffer( addr, &data, 1 );
  #if (SX1278_DEBUG==1)
  printf("WRITE REG 0x%02X = 0x%02X\r\n",addr,data);
  #endif
}

void SX1278Read( uint8_t addr, uint8_t *data )
{
  SX1278ReadBuffer( addr, data, 1 );
  #if (SX1278_DEBUG==1)
  printf("READ REG 0x%02X : 0x%02X\r\n",addr,*data);
  #endif
}

void SX1278WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1278WriteBuffer( 0, buffer, size );
}

void SX1278ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1278ReadBuffer( 0, buffer, size );
}

inline uint8_t SX1278ReadDio0( void )
{
    return HAL_GPIO_ReadPin( DIO0_IOPORT, DIO0_PIN );
}

#endif // USE_SX1278_RADIO
