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
 * \file       radio.c
 * \brief      Generic radio driver ( radio abstraction )
 *
 * \version    2.0.0 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Marc0 Xu on Jan 07 2018
 */
#include "platform.h"

#include "radio.h"

#include "sx1278.h"
#include "sx1278-HAL.h"
   
 #include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


osThreadId SX1278Handle;

tRadioDriver RadioDriver;
tRadioDriver *Radio = NULL;

uint8_t   RadioBuffer[255];
uint16_t  RadioLength;

tRadioDriver* RadioDriverInit( void )
{

    RadioDriver.Init = SX1278Init;
    RadioDriver.Reset = SX1278Reset;
    RadioDriver.StartRx = SX1278StartRx;
    RadioDriver.GetRxPacket = SX1278GetRxPacket;
    RadioDriver.SetTxPacket = SX1278SetTxPacket;
    RadioDriver.Process = SX1278Process;

    return &RadioDriver;
}

void  Radio_TxPacket(uint8_t  *Packet,uint16_t  PacketLength)
{
  Radio->SetTxPacket( Packet, PacketLength );          
}
//##############################################################################################################
void SX1278TASK_PROCESS(void const * argument)
{
  uint32_t  Last;
  BoardInit( );    
  Radio = RadioDriverInit( );    
  Radio->Init( );
  Radio_UserInit();
  Radio->StartRx( );  
  Last=HAL_GetTick();
  while(1)
  {
    switch( Radio->Process( ))
    {
      case RF_CHANNEL_ACTIVITY_DETECTED:
        __NOP();
      break;
      case RF_CHANNEL_EMPTY:
        __NOP();__NOP();
      break;
      case RF_LEN_ERROR:
        __NOP();__NOP();__NOP();
      break;
      case RF_IDLE:
          __NOP();__NOP();__NOP();__NOP();        
      break;
      case RF_BUSY:
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
      break;
      
      case RF_RX_TIMEOUT:
        Radio_RxTimeout();
        break;
      case RF_RX_DONE:
        Radio->GetRxPacket( RadioBuffer, ( uint16_t* )&RadioLength );
        if( RadioLength > 0 )
        {
          Radio_RxDone(RadioBuffer,RadioLength);
        }        
        break;
      case RF_TX_DONE:    
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        Radio->StartRx( );
        break;
      case RF_TX_TIMEOUT:
        
      break;
      default:
        break;
    }    
    osDelay(1);
    if(HAL_GetTick()-Last < 100)
    {
      Last = HAL_GetTick();
      Radio_Misc_Per100ms();
    }
  }
  
}

//##############################################################################################################

void  Radio_Init(osPriority Priority)
{
  osThreadDef(SX1278TASK, SX1278TASK_PROCESS, Priority, 0, 256);
  SX1278Handle = osThreadCreate(osThread(SX1278TASK), NULL);
}
//##############################################################################################################
