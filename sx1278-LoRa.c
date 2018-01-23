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
 * \file       sx1278-LoRa.c
 * \brief      SX1278 RF chip driver mode LoRa
 *
 * \version    2.0.0 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Marc0 Xu on Jan 07 2018
 */
#include <string.h>

#include "platform.h"

#if defined( USE_SX1278_RADIO )

#include "radio.h"

#include "sx1278-Hal.h"
#include "sx1278.h"

#include "sx1278-LoRaMisc.h"
#include "sx1278-LoRa.h"

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0

/*!
 * Frequency hopping frequencies table
 */
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

// Default settings
tLoRaSettings LoRaSettings =
{
    434000000,        // RFFrequency
    20,               // Power
    9,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    7,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    true,             // CrcOn [0: OFF, 1: ON]
    false,            // ImplicitHeaderOn [0: OFF, 1: ON]
    1,                // RxSingleOn [0: Continuous, 1 Single]
    0,                // FreqHopOn [0: OFF, 1: ON]
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    128,              // PayloadLength (used for implicit header mode)
};

/*!
 * SX1278 LoRa registers variable
 */
tSX1278LR* SX1278LR;

/*!
 * Local RF buffer for communication support
 */
static uint8_t RFBuffer[RF_BUFFER_SIZE];

/*!
 * RF state machine variable
 */
static uint8_t RFLRState = RFLR_STATE_IDLE;

/*!
 * Rx management support variables
 */
static uint16_t RxPacketSize = 0;
static int8_t RxPacketSnrEstimate;
static double RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;
/*!
 * PacketTimeout Stores the Rx window time value for packet reception
 */
static uint32_t PacketTimeout;

/*!
 * Tx management support variables
 */
static uint16_t TxPacketSize = 0;

uint8_t SX1278LoRaInit( void )
{
    uint8_t version=0;
	  
    RFLRState = RFLR_STATE_IDLE;

    SX1278LoRaSetDefaults( );
    
    version=SX1278LR->RegVersion;
    
    SX1278ReadBuffer( REG_LR_OPMODE, SX1278Regs + 1, 0x70 - 1 );
    
    SX1278LR->RegLna = RFLR_LNA_GAIN_G1;

    SX1278WriteBuffer( REG_LR_OPMODE, SX1278Regs + 1, 0x70 - 1 );

    // set the RF settings 
    SX1278LoRaSetRFFrequency( LoRaSettings.RFFrequency );
    SX1278LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor ); // SF6 only operates in implicit header mode.
    SX1278LoRaSetErrorCoding( LoRaSettings.ErrorCoding );
    SX1278LoRaSetPacketCrcOn( LoRaSettings.CrcOn );
    SX1278LoRaSetSignalBandwidth( LoRaSettings.SignalBw );

    SX1278LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );
    SX1278LoRaSetSymbTimeout( 0x3FF );
    SX1278LoRaSetPayloadLength( LoRaSettings.PayloadLength );
    //SX1278LoRaSetLowDatarateOptimize( true );

#if(  MODULE_SX1278RISYM == 1  )
    if( LoRaSettings.RFFrequency > 860000000 )
    {
        SX1278LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
        SX1278LoRaSetPa20dBm( false );
        LoRaSettings.Power = 14;
        SX1278LoRaSetRFPower( LoRaSettings.Power );
    }
    else
    {
        SX1278LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
        SX1278LoRaSetPa20dBm( true );
        LoRaSettings.Power = 20;
        SX1278LoRaSetRFPower( LoRaSettings.Power );
    } 

#endif

    SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    
    return version;
}

void SX1278LoRaSetDefaults( void )
{
    // REMARK: See SX1278 datasheet for modified default values.

    SX1278Read( REG_LR_VERSION, &SX1278LR->RegVersion );
}

void SX1278LoRaReset( void )
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

void SX1278LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    //static bool antennaSwitchTxOnPrev = true;
    //bool antennaSwitchTxOn = false;

    opModePrev = SX1278LR->RegOpMode & ~RFLR_OPMODE_MASK;
    

    if( opMode != opModePrev )
    {
        /*if( opMode == RFLR_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
            RXTX( antennaSwitchTxOn ); // Antenna switch control
        }*/
        SX1278LR->RegOpMode = ( SX1278LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;
        SX1278LR->RegOpMode |=RFLR_OPMODE_FREQMODE_ACCESS_LF;

        SX1278Write( REG_LR_OPMODE, SX1278LR->RegOpMode );        
    }
}

uint8_t SX1278LoRaGetOpMode( void )
{
    SX1278Read( REG_LR_OPMODE, &SX1278LR->RegOpMode );
    
    return SX1278LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

uint8_t SX1278LoRaReadRxGain( void )
{
    SX1278Read( REG_LR_LNA, &SX1278LR->RegLna );
    return( SX1278LR->RegLna >> 5 ) & 0x07;
}

double SX1278LoRaReadRssi( void )
{
    // Reads the RSSI value
    SX1278Read( REG_LR_RSSIVALUE, &SX1278LR->RegRssiValue );

    if( LoRaSettings.RFFrequency < 860000000 )  // LF
    {
        return RSSI_OFFSET_LF + ( double )SX1278LR->RegRssiValue;
    }
    else
    {
        return RSSI_OFFSET_HF + ( double )SX1278LR->RegRssiValue;
    }
}

uint8_t SX1278LoRaGetPacketRxGain( void )
{
    return RxGain;
}

int8_t SX1278LoRaGetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}

double SX1278LoRaGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

void SX1278LoRaStartRx( void )
{
    SX1278LoRaSetRFState( RFLR_STATE_RX_INIT );
}

void SX1278LoRaGetRxPacket( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

void SX1278LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    TxPacketSize = size;
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize ); 

    RFLRState = RFLR_STATE_TX_INIT;
}

uint8_t SX1278LoRaGetRFState( void )
{
    return RFLRState;
}

void SX1278LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1278 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint32_t SX1278LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    
    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        break;
    case RFLR_STATE_RX_INIT:
        
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |
                                    //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278Write( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1278LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
            SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1278LR->RegHopPeriod = 255;
        }
        
        SX1278Write( REG_LR_HOPPERIOD, SX1278LR->RegHopPeriod );
                
                                    // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
        SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected               ModeReady
        SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        
        SX1278WriteBuffer( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );
    
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {

            SX1278LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
        else // Rx continuous mode
        {
            SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxBaseAddr;
            SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
            
            SX1278LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
        }
        
        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = GET_TICK_COUNT( );
        RFLRState = RFLR_STATE_RX_RUNNING;
        break;
    case RFLR_STATE_RX_RUNNING:
        
        if( DIO0  ) // RxDone
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
                SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
            RFLRState = RFLR_STATE_RX_DONE;
        }
        if( DIO2  ) // FHSS Changed Channel
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
                SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
            // Debug
            RxGain = SX1278LoRaReadRxGain( );
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )
            {
                RFLRState = RFLR_STATE_RX_TIMEOUT;
            }
        }
        break;
    case RFLR_STATE_RX_DONE:
        SX1278Read( REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags );
        if( ( SX1278LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
        {
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
            if( LoRaSettings.RxSingleOn == true ) // Rx single mode
            {
                RFLRState = RFLR_STATE_RX_INIT;
            }
            else
            {
                RFLRState = RFLR_STATE_RX_RUNNING;
            }
            break;
        }
        
        {
            uint8_t rxSnrEstimate;
            SX1278Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
            if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                RxPacketSnrEstimate = -RxPacketSnrEstimate;
            }
            else
            {
                // Divide by 4
                RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
            }
        }
        
        SX1278Read( REG_LR_PKTRSSIVALUE, &SX1278LR->RegPktRssiValue );
    
        if( LoRaSettings.RFFrequency < 860000000 )  // LF
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = RSSI_OFFSET_LF + ( ( double )SX1278LR->RegPktRssiValue ) + RxPacketSnrEstimate;
            }
            else
            {
                RxPacketRssiValue = RSSI_OFFSET_LF + ( 1.0666 * ( ( double )SX1278LR->RegPktRssiValue ) );
            }
        }
        else                                        // HF
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = RSSI_OFFSET_HF + ( ( double )SX1278LR->RegPktRssiValue ) + RxPacketSnrEstimate;
            }
            else
            {    
                RxPacketRssiValue = RSSI_OFFSET_HF + ( 1.0666 * ( ( double )SX1278LR->RegPktRssiValue ) );
            }
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxBaseAddr;
            SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1278LR->RegPayloadLength;
                SX1278ReadFifo( RFBuffer, SX1278LR->RegPayloadLength );
            }
            else
            {
                SX1278Read( REG_LR_NBRXBYTES, &SX1278LR->RegNbRxBytes );
                RxPacketSize = SX1278LR->RegNbRxBytes;
                SX1278ReadFifo( RFBuffer, SX1278LR->RegNbRxBytes );
            }
        }
        else // Rx continuous mode
        {
            SX1278Read( REG_LR_FIFORXCURRENTADDR, &SX1278LR->RegFifoRxCurrentAddr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1278LR->RegPayloadLength;
                SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxCurrentAddr;
                SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
                SX1278ReadFifo( RFBuffer, SX1278LR->RegPayloadLength );
            }
            else
            {
                SX1278Read( REG_LR_NBRXBYTES, &SX1278LR->RegNbRxBytes );
                RxPacketSize = SX1278LR->RegNbRxBytes;
                SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxCurrentAddr;
                SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
                SX1278ReadFifo( RFBuffer, SX1278LR->RegNbRxBytes );
            }
        }
        
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            RFLRState = RFLR_STATE_RX_INIT;
        }
        else // Rx continuous mode
        {
            RFLRState = RFLR_STATE_RX_RUNNING;
        }
        result = RF_RX_DONE;
        break;
    case RFLR_STATE_RX_TIMEOUT:
        RFLRState = RFLR_STATE_RX_INIT;
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:

        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1278LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
            SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1278LR->RegHopPeriod = 0;
        }
        SX1278Write( REG_LR_HOPPERIOD, SX1278LR->RegHopPeriod );
        SX1278Write( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );

        // Initializes the payload size
        SX1278LR->RegPayloadLength = TxPacketSize;
        SX1278Write( REG_LR_PAYLOADLENGTH, SX1278LR->RegPayloadLength );
        
        SX1278LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
        SX1278Write( REG_LR_FIFOTXBASEADDR, SX1278LR->RegFifoTxBaseAddr );

        SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoTxBaseAddr;
        SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
        
        // Write payload buffer to LORA modem
        SX1278WriteFifo( RFBuffer, SX1278LR->RegPayloadLength );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
        SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
        SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
        
        SX1278WriteBuffer( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );

        SX1278LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

        RFLRState = RFLR_STATE_TX_RUNNING;
        break;
    case RFLR_STATE_TX_RUNNING:
        if( DIO0  ) // TxDone
        {
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
            RFLRState = RFLR_STATE_TX_DONE;   
        }
        if( DIO2  ) // FHSS Changed Channel
        {
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
                SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
        }
        break;
    case RFLR_STATE_TX_DONE:
        // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        RFLRState = RFLR_STATE_IDLE;
        result = RF_TX_DONE;
        break;
    case RFLR_STATE_CAD_INIT:    
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    
        SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        SX1278Write( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );
           
                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1278WriteBuffer( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );
            
        SX1278LoRaSetOpMode( RFLR_OPMODE_CAD );
        RFLRState = RFLR_STATE_CAD_RUNNING;
        break;
    case RFLR_STATE_CAD_RUNNING:
        if( DIO3  ) //CAD Done interrupt
        { 
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
            if( DIO4  ) // CAD Detected interrupt
            {
                // Clear Irq
                SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
                // CAD detected, we have a LoRa preamble
                RFLRState = RFLR_STATE_RX_INIT;
                result = RF_CHANNEL_ACTIVITY_DETECTED;
            } 
            else
            {    
                // The device goes in Standby Mode automatically    
                RFLRState = RFLR_STATE_IDLE;
                result = RF_CHANNEL_EMPTY;
            }
        }   
        break;
    
    default:
        break;
    } 
    return result;
}

#endif // USE_SX1278_RADIO
