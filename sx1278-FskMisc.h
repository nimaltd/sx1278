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
 * \file       sx1278-FskMisc.h
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
 * \version    2.0.B2 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Marc0 Xu on Jan 07 2018
 */
#ifndef __SX1278_FSK_MISC_H__
#define __SX1278_FSK_MISC_H__

/*!
 * \brief Writes the new RF frequency value
 *
 * \param [IN] freq New RF frequency value in [Hz]
 */
void SX1278FskSetRFFrequency( uint32_t freq );

/*!
 * \brief Reads the current RF frequency value
 *
 * \retval freq Current RF frequency value in [Hz]
 */
uint32_t SX1278FskGetRFFrequency( void );

/*!
 * \brief Calibrate RSSI and I/Q mismatch for HF 
 *
 * \retval none
 */
void SX1278FskRxCalibrate( void );

/*!
 * \brief Writes the new bitrate value
 *
 * \param [IN] bitrate New bitrate value in [bps]
 */
void SX1278FskSetBitrate( uint32_t bitrate );

/*!
 * \brief Reads the current bitrate value
 *
 * \retval bitrate Current bitrate value in [bps]
 */
uint32_t SX1278FskGetBitrate( void );

/*!
 * \brief Writes the new frequency deviation value
 *
 * \param [IN] fdev New frequency deviation value in [Hz]
 */
void SX1278FskSetFdev( uint32_t fdev );

/*!
 * \brief Reads the current frequency deviation value
 *
 * \retval fdev Current frequency deviation value in [Hz]
 */
uint32_t SX1278FskGetFdev( void );

/*!
 * \brief Writes the new RF output power value
 *
 * \param [IN] power New output power value in [dBm]
 */
void SX1278FskSetRFPower( int8_t power );

/*!
 * \brief Reads the current RF output power value
 *
 * \retval power Current output power value in [dBm]
 */
int8_t SX1278FskGetRFPower( void );

/*!
 * \brief Writes the DC offset canceller and Rx bandwidth values
 *
 * \remark For SX1278 there is no DCC setting. dccValue should be 0
 *         ie: SX1278SetDccBw( &SX1278.RegRxBw, 0, 62500 );
 *
 * \param [IN] reg Register pointer to either SX1231.RegRxBw or SX1231.RegAfcBw
 * \param [IN] dccValue New DC offset canceller value in [Hz] ( SX1231 only )
 * \param [IN] rxBwValue New Rx bandwidth value in [Hz]
 */
void SX1278FskSetDccBw( uint8_t* reg, uint32_t dccValue, uint32_t rxBwValue );

/*!
 * \brief Reads the current bandwidth setting
 *
 * \param [IN] reg Register pointer to either SX1231.RegRxBw or SX1231.RegAfcBw
 *
 * \retval bandwidth Bandwidth value
 */
uint32_t SX1278FskGetBw( uint8_t* reg );

/*!
 * \brief Enables/Disables CRC
 *
 * \param [IN] enable CRC enable/disable
 */
void SX1278FskSetPacketCrcOn( bool enable );

/*!
 * \brief Reads the current CRC Enable/Disbale value
 *
 * \retval enable Current CRC Enable/Disbale value
 */
bool SX1278FskGetPacketCrcOn( void );

/*!
 * \brief Enables/Disables AFC
 *
 * \param [IN] enable AFC enable/disable
 */
void SX1278FskSetAfcOn( bool enable );

/*!
 * \brief Reads the current AFC Enable/Disbale value
 *
 * \retval enable Current AFC Enable/Disbale value
 */
bool SX1278FskGetAfcOn( void );

/*!
 * \brief Writes the new payload length value
 *
 * \param [IN] value New payload length value
 */
void SX1278FskSetPayloadLength( uint8_t value );

/*!
 * \brief Reads the current payload length value
 *
 * \retval value Current payload length value
 */
uint8_t SX1278FskGetPayloadLength( void );

/*!
 * \brief Enables/Disables the 20 dBm PA
 *
 * \param [IN] enable [true, false]
 */
void SX1278FskSetPa20dBm( bool enale );

/*!
 * \brief Gets the current 20 dBm PA status
 *
 * \retval enable [true, false]
 */
bool SX1278FskGetPa20dBm( void );

/*!
 * \brief Set the RF Output pin 
 *
 * \param [IN] RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 */
void SX1278FskSetPAOutput( uint8_t outputPin );

/*!
 * \brief Gets the used RF Ouptu pin
 *
 * \retval RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 */
uint8_t SX1278FskGetPAOutput( void );

/*!
 * \brief Writes the new PA rise/fall time of ramp up/down value
 *
 * \param [IN] value New PaRamp value
 */
void SX1278FskSetPaRamp( uint8_t value );

/*!
 * \brief Reads the current PA rise/fall time of ramp up/down value
 *
 * \retval value Current PaRamp value
 */
uint8_t SX1278FskGetPaRamp( void );

/*!
 * \brief Applies an offset to the RSSI. Compensates board components
 *
 * \param [IN] offset Offset to be applied (+/-)
 */
void SX1278FskSetRssiOffset( int8_t offset );

/*!
 * \brief Gets the current RSSI offset.
 *
 * \retval offset Current offset (+/-)
 */
int8_t SX1278FskGetRssiOffset( void );

/*!
 * \brief Writes the new value for the preamble size
 *
 * \param [IN] size New value of pramble size
 */
void SX1278FskSetPreambleSize( uint16_t size );

/*!
 * Reads the raw temperature
 * \retval temperature New raw temperature reading in 2's complement format
 */
int8_t SX1278FskGetRawTemp( void );

/*!
 * Computes the temperature compensation factor
 * \param [IN] actualTemp Actual temperature measured by an external device
 * \retval compensationFactor Computed compensation factor
 */
int8_t SX1278FskCalibreateTemp( int8_t actualTemp );

/*!
 * Gets the actual compensated temperature
 * \param [IN] compensationFactor Return value of the calibration function
 * \retval New compensated temperature value
 */
int8_t SX1278FskGetTemp( int8_t compensationFactor );

#endif //__SX1278_FSK_MISC_H__
