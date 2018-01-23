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
 * \file       sx1278-FskMisc.c
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
 * Last modified by Marc0 Xu on Jan 07 2018
 */
#include <math.h>

#include "platform.h"

#if defined( USE_SX1278_RADIO )

#include "sx1278-Hal.h"
#include "sx1278.h"

#include "sx1278-Fsk.h"
#include "sx1278-FskMisc.h"

extern tFskSettings FskSettings;

void SX1278FskSetRFFrequency( uint32_t freq )
{
    FskSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1278->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1278->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1278->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    SX1278WriteBuffer( REG_FRFMSB, &SX1278->RegFrfMsb, 3 );
}

uint32_t SX1278FskGetRFFrequency( void )
{
    SX1278ReadBuffer( REG_FRFMSB, &SX1278->RegFrfMsb, 3 );
    FskSettings.RFFrequency = ( ( uint32_t )SX1278->RegFrfMsb << 16 ) | ( ( uint32_t )SX1278->RegFrfMid << 8 ) | ( ( uint32_t )SX1278->RegFrfLsb );
    FskSettings.RFFrequency = ( uint32_t )( ( double )FskSettings.RFFrequency * ( double )FREQ_STEP );

    return FskSettings.RFFrequency;
}

void SX1278FskRxCalibrate( void )
{
    // the function RadioRxCalibrate is called just after the reset so all register are at their default values
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // save register values;
    SX1278Read( REG_PACONFIG, &regPaConfigInitVal );
    initialFreq = SX1278FskGetRFFrequency( );

    // Cut the PA just in case
    SX1278->RegPaConfig = 0x00; // RFO output, power = -1 dBm
    SX1278Write( REG_PACONFIG, SX1278->RegPaConfig );

    // Set Frequency in HF band
    SX1278FskSetRFFrequency( 860000000 );

    // Rx chain re-calibration workaround
    SX1278Read( REG_IMAGECAL, &SX1278->RegImageCal );    
    SX1278->RegImageCal = ( SX1278->RegImageCal & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START;
    SX1278Write( REG_IMAGECAL, SX1278->RegImageCal );

    SX1278Read( REG_IMAGECAL, &SX1278->RegImageCal );
    // rx_cal_run goes low when calibration in finished
    while( ( SX1278->RegImageCal & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
        SX1278Read( REG_IMAGECAL, &SX1278->RegImageCal );
    }

    // reload saved values into the registers
    SX1278->RegPaConfig = regPaConfigInitVal;
    SX1278Write( REG_PACONFIG, SX1278->RegPaConfig );

    SX1278FskSetRFFrequency( initialFreq );

}

void SX1278FskSetBitrate( uint32_t bitrate )
{
    FskSettings.Bitrate = bitrate;
    
    bitrate = ( uint16_t )( ( double )XTAL_FREQ / ( double )bitrate );
    SX1278->RegBitrateMsb    = ( uint8_t )( bitrate >> 8 );
    SX1278->RegBitrateLsb    = ( uint8_t )( bitrate & 0xFF );
    SX1278WriteBuffer( REG_BITRATEMSB, &SX1278->RegBitrateMsb, 2 );    
}

uint32_t SX1278FskGetBitrate( void )
{
    SX1278ReadBuffer( REG_BITRATEMSB, &SX1278->RegBitrateMsb, 2 );
    FskSettings.Bitrate = ( ( ( uint32_t )SX1278->RegBitrateMsb << 8 ) | ( ( uint32_t )SX1278->RegBitrateLsb ) );
    FskSettings.Bitrate = ( uint16_t )( ( double )XTAL_FREQ / ( double )FskSettings.Bitrate );

    return FskSettings.Bitrate;
}

void SX1278FskSetFdev( uint32_t fdev )
{
    FskSettings.Fdev = fdev;

    SX1278Read( REG_FDEVMSB, &SX1278->RegFdevMsb ); 

    fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
    SX1278->RegFdevMsb    = ( ( SX1278->RegFdevMsb & RF_FDEVMSB_FDEV_MASK ) | ( ( ( uint8_t )( fdev >> 8 ) ) & ~RF_FDEVMSB_FDEV_MASK ) );
    SX1278->RegFdevLsb    = ( uint8_t )( fdev & 0xFF );
    SX1278WriteBuffer( REG_FDEVMSB, &SX1278->RegFdevMsb, 2 );    
}

uint32_t SX1278FskGetFdev( void )
{
    SX1278ReadBuffer( REG_FDEVMSB, &SX1278->RegFdevMsb, 2 );
    FskSettings.Fdev = ( ( ( uint32_t )( ( SX1278->RegFdevMsb << 8 ) & ~RF_FDEVMSB_FDEV_MASK ) ) | ( ( uint32_t )SX1278->RegFdevLsb ) );
    FskSettings.Fdev = ( uint16_t )( ( double )FskSettings.Fdev * ( double )FREQ_STEP );

    return FskSettings.Fdev;
}

void SX1278FskSetRFPower( int8_t power )
{
    SX1278Read( REG_PACONFIG, &SX1278->RegPaConfig );
    SX1278Read( REG_PADAC, &SX1278->RegPaDac );
    
    if( ( SX1278->RegPaConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1278->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1278->RegPaConfig = ( SX1278->RegPaConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278->RegPaConfig = ( SX1278->RegPaConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
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
            SX1278->RegPaConfig = ( SX1278->RegPaConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278->RegPaConfig = ( SX1278->RegPaConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
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
        SX1278->RegPaConfig = ( SX1278->RegPaConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1278->RegPaConfig = ( SX1278->RegPaConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1278Write( REG_PACONFIG, SX1278->RegPaConfig );
    FskSettings.Power = power;
}

int8_t SX1278FskGetRFPower( void )
{
    SX1278Read( REG_PACONFIG, &SX1278->RegPaConfig );
    SX1278Read( REG_PADAC, &SX1278->RegPaDac );

    if( ( SX1278->RegPaConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1278->RegPaDac & 0x07 ) == 0x07 )
        {
            FskSettings.Power = 5 + ( SX1278->RegPaConfig & ~RF_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            FskSettings.Power = 2 + ( SX1278->RegPaConfig & ~RF_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        FskSettings.Power = -1 + ( SX1278->RegPaConfig & ~RF_PACONFIG_OUTPUTPOWER_MASK );
    }
    return FskSettings.Power;
}

/*!
 * \brief Computes the Rx bandwidth with the mantisse and exponent
 *
 * \param [IN] mantisse Mantisse of the bandwidth value
 * \param [IN] exponent Exponent of the bandwidth value
 * \retval bandwidth Computed bandwidth
 */
static uint32_t SX1278FskComputeRxBw( uint8_t mantisse, uint8_t exponent )
{
    // rxBw
    if( ( SX1278->RegOpMode & RF_OPMODE_MODULATIONTYPE_FSK ) == RF_OPMODE_MODULATIONTYPE_FSK )
    {
        return ( uint32_t )( ( double )XTAL_FREQ / ( mantisse * ( double )pow( 2, exponent + 2 ) ) );
    }
    else
    {
        return ( uint32_t )( ( double )XTAL_FREQ / ( mantisse * ( double )pow( 2, exponent + 3 ) ) );
    }
}

/*!
 * \brief Computes the mantisse and exponent from the bandwitdh value
 *
 * \param [IN] rxBwValue Bandwidth value
 * \param [OUT] mantisse Mantisse of the bandwidth value
 * \param [OUT] exponent Exponent of the bandwidth value
 */
static void SX1278FskComputeRxBwMantExp( uint32_t rxBwValue, uint8_t* mantisse, uint8_t* exponent )
{
    uint8_t tmpExp = 0;
    uint8_t tmpMant = 0;

    double tmpRxBw = 0;
    double rxBwMin = 10e6;

    for( tmpExp = 0; tmpExp < 8; tmpExp++ )
    {
        for( tmpMant = 16; tmpMant <= 24; tmpMant += 4 )
        {
            if( ( SX1278->RegOpMode & RF_OPMODE_MODULATIONTYPE_FSK ) == RF_OPMODE_MODULATIONTYPE_FSK )
            {
                tmpRxBw = ( double )XTAL_FREQ / ( tmpMant * ( double )pow( 2, tmpExp + 2 ) );
            }
            else
            {
                tmpRxBw = ( double )XTAL_FREQ / ( tmpMant * ( double )pow( 2, tmpExp + 3 ) );
            }
            if( fabs( tmpRxBw - rxBwValue ) < rxBwMin )
            {
                rxBwMin = fabs( tmpRxBw - rxBwValue );
                *mantisse = tmpMant;
                *exponent = tmpExp;
            }
        }
    }
}

void SX1278FskSetDccBw( uint8_t* reg, uint32_t dccValue, uint32_t rxBwValue )
{
    uint8_t mantisse = 0;
    uint8_t exponent = 0;
    
    if( reg == &SX1278->RegRxBw )
    {
        *reg = ( uint8_t )dccValue & 0x60;
    }
    else
    {
        *reg = 0;
    }

    SX1278FskComputeRxBwMantExp( rxBwValue, &mantisse, &exponent );

    switch( mantisse )
    {
        case 16:
            *reg |= ( uint8_t )( 0x00 | ( exponent & 0x07 ) );
            break;
        case 20:
            *reg |= ( uint8_t )( 0x08 | ( exponent & 0x07 ) );
            break;
        case 24:
            *reg |= ( uint8_t )( 0x10 | ( exponent & 0x07 ) );
            break;
        default:
            // Something went terribely wrong
            break;
    }

    if( reg == &SX1278->RegRxBw )
    {
        SX1278Write( REG_RXBW, *reg );
        FskSettings.RxBw = rxBwValue;
    }
    else
    {
        SX1278Write( REG_AFCBW, *reg );
        FskSettings.RxBwAfc = rxBwValue;
    }
}

uint32_t SX1278FskGetBw( uint8_t* reg )
{
    uint32_t rxBwValue = 0;
    uint8_t mantisse = 0;
    switch( ( *reg & 0x18 ) >> 3 )
    {
        case 0:
            mantisse = 16;
            break;
        case 1:
            mantisse = 20;
            break;
        case 2:
            mantisse = 24;
            break;
        default:
            break;
    }
    rxBwValue = SX1278FskComputeRxBw( mantisse, ( uint8_t )*reg & 0x07 );
    if( reg == &SX1278->RegRxBw )
    {
        return FskSettings.RxBw = rxBwValue;
    }
    else
    {
        return FskSettings.RxBwAfc = rxBwValue;
    }
}

void SX1278FskSetPacketCrcOn( bool enable )
{
    SX1278Read( REG_PACKETCONFIG1, &SX1278->RegPacketConfig1 );
    SX1278->RegPacketConfig1 = ( SX1278->RegPacketConfig1 & RF_PACKETCONFIG1_CRC_MASK ) | ( enable << 4 );
    SX1278Write( REG_PACKETCONFIG1, SX1278->RegPacketConfig1 );
    FskSettings.CrcOn = enable;
}

bool SX1278FskGetPacketCrcOn( void )
{
    SX1278Read( REG_PACKETCONFIG1, &SX1278->RegPacketConfig1 );
    FskSettings.CrcOn = ( SX1278->RegPacketConfig1 & RF_PACKETCONFIG1_CRC_ON ) >> 4;
    return FskSettings.CrcOn;
}

void SX1278FskSetAfcOn( bool enable )
{
    SX1278Read( REG_RXCONFIG, &SX1278->RegRxConfig );
    SX1278->RegRxConfig = ( SX1278->RegRxConfig & RF_RXCONFIG_AFCAUTO_MASK ) | ( enable << 4 );
    SX1278Write( REG_RXCONFIG, SX1278->RegRxConfig );
    FskSettings.AfcOn = enable;
}

bool SX1278FskGetAfcOn( void )
{
    SX1278Read( REG_RXCONFIG, &SX1278->RegRxConfig );
    FskSettings.AfcOn = ( SX1278->RegRxConfig & RF_RXCONFIG_AFCAUTO_ON ) >> 4;
    return FskSettings.AfcOn;
}

void SX1278FskSetPayloadLength( uint8_t value )
{
    SX1278->RegPayloadLength = value;
    SX1278Write( REG_PAYLOADLENGTH, SX1278->RegPayloadLength );
    FskSettings.PayloadLength = value;
}

uint8_t SX1278FskGetPayloadLength( void )
{
    SX1278Read( REG_PAYLOADLENGTH, &SX1278->RegPayloadLength );
    FskSettings.PayloadLength = SX1278->RegPayloadLength;
    return FskSettings.PayloadLength;
}

void SX1278FskSetPa20dBm( bool enale )
{
    SX1278Read( REG_PADAC, &SX1278->RegPaDac );
    SX1278Read( REG_PACONFIG, &SX1278->RegPaConfig );

    if( ( SX1278->RegPaConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {    
        if( enale == true )
        {
            SX1278->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1278->RegPaDac = 0x84;
    }
    SX1278Write( REG_PADAC, SX1278->RegPaDac );
}

bool SX1278FskGetPa20dBm( void )
{
    SX1278Read( REG_PADAC, &SX1278->RegPaDac );
    
    return ( ( SX1278->RegPaDac & 0x07 ) == 0x07 ) ? true : false;
}

void SX1278FskSetPAOutput( uint8_t outputPin )
{
    SX1278Read( REG_PACONFIG, &SX1278->RegPaConfig );
    SX1278->RegPaConfig = (SX1278->RegPaConfig & RF_PACONFIG_PASELECT_MASK ) | outputPin;
    SX1278Write( REG_PACONFIG, SX1278->RegPaConfig );
}

uint8_t SX1278FskGetPAOutput( void )
{
    SX1278Read( REG_PACONFIG, &SX1278->RegPaConfig );
    return SX1278->RegPaConfig & ~RF_PACONFIG_PASELECT_MASK;
}


void SX1278FskSetPaRamp( uint8_t value )
{
    SX1278Read( REG_PARAMP, &SX1278->RegPaRamp );
    SX1278->RegPaRamp = ( SX1278->RegPaRamp & RF_PARAMP_MASK ) | ( value & ~RF_PARAMP_MASK );
    SX1278Write( REG_PARAMP, SX1278->RegPaRamp );
}

uint8_t SX1278FskGetPaRamp( void )
{
    SX1278Read( REG_PARAMP, &SX1278->RegPaRamp );
    return SX1278->RegPaRamp & ~RF_PARAMP_MASK;
}

void SX1278FskSetRssiOffset( int8_t offset )
{
    SX1278Read( REG_RSSICONFIG, &SX1278->RegRssiConfig );
    if( offset < 0 )
    {
        offset = ( ~offset & 0x1F );
        offset += 1;
        offset = -offset;
    }
    SX1278->RegRssiConfig |= ( uint8_t )( ( offset & 0x1F ) << 3 );
    SX1278Write( REG_RSSICONFIG, SX1278->RegRssiConfig );
}

int8_t SX1278FskGetRssiOffset( void )
{
    SX1278Read( REG_RSSICONFIG, &SX1278->RegRssiConfig );
    int8_t offset = SX1278->RegRssiConfig >> 3;
    if( ( offset & 0x10 ) == 0x10 )
    {
        offset = ( ~offset & 0x1F );
        offset += 1;
        offset = -offset;
    }
    return offset;
}

int8_t SX1278FskGetRawTemp( void )
{
    int8_t temp = 0;
    uint8_t previousOpMode;
    uint32_t startTick;
    
    // Enable Temperature reading
    SX1278Read( REG_IMAGECAL, &SX1278->RegImageCal );
    SX1278->RegImageCal = ( SX1278->RegImageCal & RF_IMAGECAL_TEMPMONITOR_MASK ) | RF_IMAGECAL_TEMPMONITOR_ON;
    SX1278Write( REG_IMAGECAL, SX1278->RegImageCal );

    // save current Op Mode
    SX1278Read( REG_OPMODE, &SX1278->RegOpMode );
    previousOpMode = SX1278->RegOpMode;

    // put device in FSK RxSynth
    SX1278->RegOpMode = RF_OPMODE_SYNTHESIZER_RX;
    SX1278Write( REG_OPMODE, SX1278->RegOpMode );

    // Wait 1ms
    startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 1 ) );  

    // Disable Temperature reading
    SX1278Read( REG_IMAGECAL, &SX1278->RegImageCal );
    SX1278->RegImageCal = ( SX1278->RegImageCal & RF_IMAGECAL_TEMPMONITOR_MASK ) | RF_IMAGECAL_TEMPMONITOR_OFF;
    SX1278Write( REG_IMAGECAL, SX1278->RegImageCal );

    // Read temperature
    SX1278Read( REG_TEMP, &SX1278->RegTemp );
    
    temp = SX1278->RegTemp & 0x7F;
    
    if( ( SX1278->RegTemp & 0x80 ) == 0x80 )
    {
        temp *= -1;
    }

    // Reload previous Op Mode
    SX1278Write( REG_OPMODE, previousOpMode );

    return temp;
}

int8_t SX1278FskCalibreateTemp( int8_t actualTemp )
{
    return actualTemp - SX1278FskGetRawTemp( );
}

int8_t SX1278FskGetTemp( int8_t compensationFactor )
{
    return SX1278FskGetRawTemp( ) + compensationFactor;
}

#endif // USE_SX1278_RADIO
