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
 * \file       sx1278.c
 * \brief      SX1278 RF chip driver
 *
 * \version    2.0.0 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Marc0 Xu on Jun 07 2018
 */
#include "platform.h"
#include "radio.h"

#if defined( USE_SX1278_RADIO )

#include "sx1278.h"

#include "sx1278-Hal.h"
#include "sx1278-Fsk.h"
#include "sx1278-LoRa.h"

/*!
 * SX1278 registers variable
 */
uint8_t SX1278Regs[0x70];

static bool LoRaOn = false;
static bool LoRaOnState = false;

void SX1278Init( void )
{
    // Initialize FSK and LoRa registers structure
    SX1278 = ( tSX1278* )SX1278Regs;
    SX1278LR = ( tSX1278LR* )SX1278Regs;

    SX1278InitIo( );
    
    SX1278Reset( );

    // REMARK: After radio reset the default modem is FSK

#if ( LORA == 0 ) 

    LoRaOn = false;
    SX1278SetLoRaOn( LoRaOn );
    // Initialize FSK modem
    SX1278FskInit( );

#else

    LoRaOn = true;
    SX1278SetLoRaOn( LoRaOn );
    // Initialize LoRa modem
    SX1278LoRaInit( );
    
#endif

}

void SX1278Reset( void )
{
    //SX1278SetReset( RADIO_RESET_ON );
    
    // Wait 1ms
    //uint32_t startTick = GET_TICK_COUNT( );
    //while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 1 ) );    

    //SX1278SetReset( RADIO_RESET_OFF );
    
    // Wait 6ms
    //startTick = GET_TICK_COUNT( );
    //while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 6 ) );    
}

void SX1278SetLoRaOn( bool enable )
{
    if( LoRaOnState == enable )
    {
        return;
    }
    LoRaOnState = enable;
    LoRaOn = enable;

    if( LoRaOn == true )
    {
        SX1278LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1278LR->RegOpMode = ( SX1278LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
        SX1278Write( REG_LR_OPMODE, SX1278LR->RegOpMode );
        
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
                                        // RxDone               RxTimeout                   FhssChangeChannel           CadDone
        SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // CadDetected          ModeReady
        SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1278WriteBuffer( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );
        
        SX1278ReadBuffer( REG_LR_OPMODE, SX1278Regs + 1, 0x70 - 1 );
    }
    else
    {
        SX1278LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1278LR->RegOpMode = ( SX1278LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        SX1278Write( REG_LR_OPMODE, SX1278LR->RegOpMode );
        
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
        
        SX1278ReadBuffer( REG_OPMODE, SX1278Regs + 1, 0x70 - 1 );
    }
}

bool SX1278GetLoRaOn( void )
{
    return LoRaOn;
}

void SX1278SetOpMode( uint8_t opMode )
{
    if( LoRaOn == false )
    {
        SX1278FskSetOpMode( opMode );
    }
    else
    {
        SX1278LoRaSetOpMode( opMode );
    }
}

uint8_t SX1278GetOpMode( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskGetOpMode( );
    }
    else
    {
        return SX1278LoRaGetOpMode( );
    }
}

double SX1278ReadRssi( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskReadRssi( );
    }
    else
    {
        return SX1278LoRaReadRssi( );
    }
}

uint8_t SX1278ReadRxGain( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskReadRxGain( );
    }
    else
    {
        return SX1278LoRaReadRxGain( );
    }
}

uint8_t SX1278GetPacketRxGain( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskGetPacketRxGain(  );
    }
    else
    {
        return SX1278LoRaGetPacketRxGain(  );
    }
}

int8_t SX1278GetPacketSnr( void )
{
    if( LoRaOn == false )
    {
         while( 1 )
         {
             // Useless in FSK mode
             // Block program here
         }
    }
    else
    {
        return SX1278LoRaGetPacketSnr(  );
    }
}

double SX1278GetPacketRssi( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskGetPacketRssi(  );
    }
    else
    {
        return SX1278LoRaGetPacketRssi( );
    }
}

uint32_t SX1278GetPacketAfc( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskGetPacketAfc(  );
    }
    else
    {
         while( 1 )
         {
             // Useless in LoRa mode
             // Block program here
         }
    }
}

void SX1278StartRx( void )
{
    if( LoRaOn == false )
    {
        SX1278FskSetRFState( RF_STATE_RX_INIT );
    }
    else
    {
        SX1278LoRaSetRFState( RFLR_STATE_RX_INIT );
    }
}

void SX1278GetRxPacket( void *buffer, uint16_t *size )
{
    if( LoRaOn == false )
    {
        SX1278FskGetRxPacket( buffer, size );
    }
    else
    {
        SX1278LoRaGetRxPacket( buffer, size );
    }
}

void SX1278SetTxPacket( const void *buffer, uint16_t size )
{
    if( LoRaOn == false )
    {
        SX1278FskSetTxPacket( buffer, size );
    }
    else
    {
        SX1278LoRaSetTxPacket( buffer, size );
    }
}

uint8_t SX1278GetRFState( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskGetRFState( );
    }
    else
    {
        return SX1278LoRaGetRFState( );
    }
}

void SX1278SetRFState( uint8_t state )
{
    if( LoRaOn == false )
    {
        SX1278FskSetRFState( state );
    }
    else
    {
        SX1278LoRaSetRFState( state );
    }
}

uint32_t SX1278Process( void )
{
    if( LoRaOn == false )
    {
        return SX1278FskProcess( );
    }
    else
    {
        return SX1278LoRaProcess( );
    }
}

#endif // USE_SX1278_RADIO
