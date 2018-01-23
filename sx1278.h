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
 * \file       sx1278.h
 * \brief      SX1278 RF chip driver
 *
 * \version    2.0.B2 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Marc0 Xu on Jan 06 2018
 */
#ifndef __SX1278_H__
#define __SX1278_H__

/*!
 * \brief SX1278 registers array
 */
extern uint8_t SX1278Regs[0x70];

/*!
 * \brief Enables LoRa modem or FSK modem
 *
 * \param [IN] opMode New operating mode
 */
void SX1278SetLoRaOn( bool enable );

/*!
 * \brief Gets the LoRa modem state
 *
 * \retval LoraOn Current LoRa modem mode
 */
bool SX1278GetLoRaOn( void );

/*!
 * \brief Initializes the SX1278
 */
void SX1278Init( void );

/*!
 * \brief Resets the SX1278
 */
void SX1278Reset( void );

/*!
 * \brief Sets the SX1278 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1278SetOpMode( uint8_t opMode );

/*!
 * \brief Gets the SX1278 operating mode
 *
 * \retval opMode Current operating mode
 */
uint8_t SX1278GetOpMode( void );

/*!
 * \brief Reads the current Rx gain setting
 *
 * \retval rxGain Current gain setting
 */
uint8_t SX1278ReadRxGain( void );

/*!
 * \brief Trigs and reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
double SX1278ReadRssi( void );

/*!
 * \brief Gets the Rx gain value measured while receiving the packet
 *
 * \retval rxGainValue Current Rx gain value
 */
uint8_t SX1278GetPacketRxGain( void );

/*!
 * \brief Gets the SNR value measured while receiving the packet
 *
 * \retval snrValue Current SNR value in [dB]
 */
int8_t SX1278GetPacketSnr( void );

/*!
 * \brief Gets the RSSI value measured while receiving the packet
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
double SX1278GetPacketRssi( void );

/*!
 * \brief Gets the AFC value measured while receiving the packet
 *
 * \retval afcValue Current AFC value in [Hz]
 */
uint32_t SX1278GetPacketAfc( void );

/*!
 * \brief Sets the radio in Rx mode. Waiting for a packet
 */
void SX1278StartRx( void );

/*!
 * \brief Gets a copy of the current received buffer
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void SX1278GetRxPacket( void *buffer, uint16_t *size );

/*!
 * \brief Sets a copy of the buffer to be transmitted and starts the
 *        transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void SX1278SetTxPacket( const void *buffer, uint16_t size );

/*!
 * \brief Gets the current RFState
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint8_t SX1278GetRFState( void );

/*!
 * \brief Sets the new state of the RF state machine
 *
 * \param [IN]: state New RF state machine state
 */
void SX1278SetRFState( uint8_t state );

/*!
 * \brief Process the Rx and Tx state machines depending on the
 *       SX1278 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint32_t SX1278Process( void );

#endif //__SX1278_H__
