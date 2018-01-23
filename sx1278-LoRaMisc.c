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
 * \file       sx1278-LoRaMisc.c
 * \brief      SX1278 RF chip high level functions driver
 *
 * \remark     Optional support functions.
 *             These functions are defined only to easy the change of the
 *             parameters.
 *             For a final firmware the radio parameters will be known so
 *             there is no need to support all possible parameters.
 *             Removing these functions will greatly reduce the final firmware
 *             size.
 *
 * \version    2.0.0 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Marc0 Xu on Jun 07 2018
 */
#include "platform.h"

#if defined( USE_SX1278_RADIO )

#include "sx1278-Hal.h"
#include "sx1278.h"

#include "sx1278-LoRa.h"
#include "sx1278-LoRaMisc.h"

/*!
 * SX1278 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

extern tLoRaSettings LoRaSettings;

void SX1278LoRaSetRFFrequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1278LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1278LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1278LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    SX1278WriteBuffer( REG_LR_FRFMSB, &SX1278LR->RegFrfMsb, 3 );
}

uint32_t SX1278LoRaGetRFFrequency( void )
{
    SX1278ReadBuffer( REG_LR_FRFMSB, &SX1278LR->RegFrfMsb, 3 );
    LoRaSettings.RFFrequency = ( ( uint32_t )SX1278LR->RegFrfMsb << 16 ) | ( ( uint32_t )SX1278LR->RegFrfMid << 8 ) | ( ( uint32_t )SX1278LR->RegFrfLsb );
    LoRaSettings.RFFrequency = ( uint32_t )( ( double )LoRaSettings.RFFrequency * ( double )FREQ_STEP );

    return LoRaSettings.RFFrequency;
}

void SX1278LoRaSetRFPower( int8_t power )
{
    SX1278Read( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );
    SX1278Read( REG_LR_PADAC, &SX1278LR->RegPaDac );
    
    if( ( SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1278LR->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1278LR->RegPaConfig = ( SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1278Write( REG_LR_PACONFIG, SX1278LR->RegPaConfig );
    LoRaSettings.Power = power;
}

int8_t SX1278LoRaGetRFPower( void )
{
    SX1278Read( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );
    SX1278Read( REG_LR_PADAC, &SX1278LR->RegPaDac );

    if( ( SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1278LR->RegPaDac & 0x07 ) == 0x07 )
        {
            LoRaSettings.Power = 5 + ( SX1278LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            LoRaSettings.Power = 2 + ( SX1278LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        LoRaSettings.Power = -1 + ( SX1278LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
    }
    return LoRaSettings.Power;
}

void SX1278LoRaSetSignalBandwidth( uint8_t bw )
{
    SX1278Read( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    SX1278LR->RegModemConfig1 = ( SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    SX1278Write( REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}

uint8_t SX1278LoRaGetSignalBandwidth( void )
{
    SX1278Read( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    LoRaSettings.SignalBw = ( SX1278LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_BW_MASK ) >> 4;
    return LoRaSettings.SignalBw;
}

void SX1278LoRaSetSpreadingFactor( uint8_t factor )
{

    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    if( factor == 6 )
    {
        SX1278LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1278LoRaSetNbTrigPeaks( 3 );
    }

    SX1278Read( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2 );    
    SX1278LR->RegModemConfig2 = ( SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    SX1278Write( REG_LR_MODEMCONFIG2, SX1278LR->RegModemConfig2 );    
    LoRaSettings.SpreadingFactor = factor;
}

uint8_t SX1278LoRaGetSpreadingFactor( void )
{
    SX1278Read( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2 );   
    LoRaSettings.SpreadingFactor = ( SX1278LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;
    return LoRaSettings.SpreadingFactor;
}

void SX1278LoRaSetErrorCoding( uint8_t value )
{
    SX1278Read( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    SX1278LR->RegModemConfig1 = ( SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    SX1278Write( REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}

uint8_t SX1278LoRaGetErrorCoding( void )
{
    SX1278Read( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = ( SX1278LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK ) >> 1;
    return LoRaSettings.ErrorCoding;
}

void SX1278LoRaSetPacketCrcOn( bool enable )
{
    SX1278Read( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2 );
    SX1278LR->RegModemConfig2 = ( SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    SX1278Write( REG_LR_MODEMCONFIG2, SX1278LR->RegModemConfig2 );
    LoRaSettings.CrcOn = enable;
}

void SX1278LoRaSetPreambleLength( uint16_t value )
{
    SX1278ReadBuffer( REG_LR_PREAMBLEMSB, &SX1278LR->RegPreambleMsb, 2 );

    SX1278LR->RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    SX1278LR->RegPreambleLsb = value & 0xFF;
    SX1278WriteBuffer( REG_LR_PREAMBLEMSB, &SX1278LR->RegPreambleMsb, 2 );
}

uint16_t SX1278LoRaGetPreambleLength( void )
{
    SX1278ReadBuffer( REG_LR_PREAMBLEMSB, &SX1278LR->RegPreambleMsb, 2 );
    return ( ( SX1278LR->RegPreambleMsb & 0x00FF ) << 8 ) | SX1278LR->RegPreambleLsb;
}

bool SX1278LoRaGetPacketCrcOn( void )
{
    SX1278Read( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2 );
    LoRaSettings.CrcOn = ( SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON ) >> 1;
    return LoRaSettings.CrcOn;
}

void SX1278LoRaSetImplicitHeaderOn( bool enable )
{
    SX1278Read( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    SX1278LR->RegModemConfig1 = ( SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    SX1278Write( REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}

bool SX1278LoRaGetImplicitHeaderOn( void )
{
    SX1278Read( REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = ( SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_ON );
    return LoRaSettings.ImplicitHeaderOn;
}

void SX1278LoRaSetRxSingleOn( bool enable )
{
    LoRaSettings.RxSingleOn = enable;
}

bool SX1278LoRaGetRxSingleOn( void )
{
    return LoRaSettings.RxSingleOn;
}

void SX1278LoRaSetFreqHopOn( bool enable )
{
    LoRaSettings.FreqHopOn = enable;
}

bool SX1278LoRaGetFreqHopOn( void )
{
    return LoRaSettings.FreqHopOn;
}

void SX1278LoRaSetHopPeriod( uint8_t value )
{
    SX1278LR->RegHopPeriod = value;
    SX1278Write( REG_LR_HOPPERIOD, SX1278LR->RegHopPeriod );
    LoRaSettings.HopPeriod = value;
}

uint8_t SX1278LoRaGetHopPeriod( void )
{
    SX1278Read( REG_LR_HOPPERIOD, &SX1278LR->RegHopPeriod );
    LoRaSettings.HopPeriod = SX1278LR->RegHopPeriod;
    return LoRaSettings.HopPeriod;
}

void SX1278LoRaSetTxPacketTimeout( uint32_t value )
{
    LoRaSettings.TxPacketTimeout = value;
}

uint32_t SX1278LoRaGetTxPacketTimeout( void )
{
    return LoRaSettings.TxPacketTimeout;
}

void SX1278LoRaSetRxPacketTimeout( uint32_t value )
{
    LoRaSettings.RxPacketTimeout = value;
}

uint32_t SX1278LoRaGetRxPacketTimeout( void )
{
    return LoRaSettings.RxPacketTimeout;
}

void SX1278LoRaSetPayloadLength( uint8_t value )
{
    SX1278LR->RegPayloadLength = value;
    SX1278Write( REG_LR_PAYLOADLENGTH, SX1278LR->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}

uint8_t SX1278LoRaGetPayloadLength( void )
{
    SX1278Read( REG_LR_PAYLOADLENGTH, &SX1278LR->RegPayloadLength );
    LoRaSettings.PayloadLength = SX1278LR->RegPayloadLength;
    return LoRaSettings.PayloadLength;
}

void SX1278LoRaSetPa20dBm( bool enale )
{
    SX1278Read( REG_LR_PADAC, &SX1278LR->RegPaDac );
    SX1278Read( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );

    if( ( SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {    
        if( enale == true )
        {
            SX1278LR->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1278LR->RegPaDac = 0x84;
    }
    SX1278Write( REG_LR_PADAC, SX1278LR->RegPaDac );
}

bool SX1278LoRaGetPa20dBm( void )
{
    SX1278Read( REG_LR_PADAC, &SX1278LR->RegPaDac );
    
    return ( ( SX1278LR->RegPaDac & 0x07 ) == 0x07 ) ? true : false;
}

void SX1278LoRaSetPAOutput( uint8_t outputPin )
{
    SX1278Read( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );
    SX1278LR->RegPaConfig = (SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    SX1278Write( REG_LR_PACONFIG, SX1278LR->RegPaConfig );
}

uint8_t SX1278LoRaGetPAOutput( void )
{
    SX1278Read( REG_LR_PACONFIG, &SX1278LR->RegPaConfig );
    return SX1278LR->RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;
}

void SX1278LoRaSetPaRamp( uint8_t value )
{
    SX1278Read( REG_LR_PARAMP, &SX1278LR->RegPaRamp );
    SX1278LR->RegPaRamp = ( SX1278LR->RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK );
    SX1278Write( REG_LR_PARAMP, SX1278LR->RegPaRamp );
}

uint8_t SX1278LoRaGetPaRamp( void )
{
    SX1278Read( REG_LR_PARAMP, &SX1278LR->RegPaRamp );
    return SX1278LR->RegPaRamp & ~RFLR_PARAMP_MASK;
}

void SX1278LoRaSetSymbTimeout( uint16_t value )
{
    SX1278ReadBuffer( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2, 2 );

    SX1278LR->RegModemConfig2 = ( SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1278LR->RegSymbTimeoutLsb = value & 0xFF;
    SX1278WriteBuffer( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2, 2 );
}

uint16_t SX1278LoRaGetSymbTimeout( void )
{
    SX1278ReadBuffer( REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2, 2 );
    return ( ( SX1278LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) << 8 ) | SX1278LR->RegSymbTimeoutLsb;
}

void SX1278LoRaSetLowDatarateOptimize( bool enable )
{
    SX1278Read( REG_LR_MODEMCONFIG3, &SX1278LR->RegModemConfig3 );
    SX1278LR->RegModemConfig3 = ( SX1278LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    SX1278Write( REG_LR_MODEMCONFIG3, SX1278LR->RegModemConfig3 );
}

bool SX1278LoRaGetLowDatarateOptimize( void )
{
    SX1278Read( REG_LR_MODEMCONFIG3, &SX1278LR->RegModemConfig3 );
    return ( ( SX1278LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON ) >> 3 );
}

void SX1278LoRaSetNbTrigPeaks( uint8_t value )
{
    SX1278Read( 0x31, &SX1278LR->RegDetectOptimize );
    SX1278LR->RegDetectOptimize = ( SX1278LR->RegDetectOptimize & 0xF8 ) | value;
    SX1278Write( 0x31, SX1278LR->RegDetectOptimize );
}

uint8_t SX1278LoRaGetNbTrigPeaks( void )
{
    SX1278Read( 0x31, &SX1278LR->RegDetectOptimize );
    return ( SX1278LR->RegDetectOptimize & 0x07 );
}

#endif // USE_SX1278_RADIO
