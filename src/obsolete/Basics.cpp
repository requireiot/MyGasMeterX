/**
 * @file 		  Basics.cpp
 *
 * Project		: Home automation
 * Author		: Bernd Waldmann
 * Created		: 09-Jun-2021
 * Tabsize		: 4
 * 
 * This Revision: $Id: $
 */

/*
   Copyright (C) 2021 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Basic initializaton tasks that need to be done in most sensor projects.
 * 
 */


#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/boot.h>

#include <core/MySensorsCore.h>
#include <core/MyIndication.h>

#include <AvrBattery.h>
#include "stdpins.h"
#include "debugstream.h"


#if     ((F_CPU /  2L) < 200000uL)
 #define ADC_PRESCALER 1
#elif ((F_CPU /  4L) < 200000uL)
 #define ADC_PRESCALER 2
#elif ((F_CPU /  8L) < 200000uL)
 #define ADC_PRESCALER 3
#elif ((F_CPU / 16L) < 200000uL)
 #define ADC_PRESCALER 4
#elif ((F_CPU / 32L) < 200000uL)
 #define ADC_PRESCALER 5
#elif ((F_CPU / 64L) < 200000uL)
 #define ADC_PRESCALER 6
#else 
 #define ADC_PRESCALER 7
#endif

/**
 * @brief Basic initialization of peripherals, for minimum power consumption.
 * Call this from setup(), or from preHwInit(), which is called from MySensors framework
 * 
 */
void basicHwInit()
{
	// first disable ADC, then turn off in PRR
	ADCSRA 	= (ADC_PRESCALER << ADPS0)	// prescaler
			| (0 << ADIE)	// no interrupts
			| (0 << ADATE)	// no auto trigger enable
			| (0 << ADEN)	// disable ADC for now
			;
	PRR = _BV(PRADC) | _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2);

	// disable analog comparator
	ACSR |= _BV(ACD);

	// set direction and pull-ups, this is specific for ATmega328
	DDRB &= ~0b11000001;        // port B: PB1-5 used by SPI; PB6,7 are Xtal
	PORTB |= 0b00000001;        // enable pull-up

	DDRC = 0;                   // port C all input (default after reset anyway)
	PORTC = 0xFF;               // enable pull-up on all bits to save power ...
#ifdef REPORT_CLIMATE
    PULLUP_DISABLE(_I2C_SCL);   // ... except no pull-up on SDA,SCL
    PULLUP_DISABLE(_I2C_SDA);	
#endif

	DDRD = 0;                   // port D all input (default after reset anyway)
	PORTD = 0xFF;               // enable pull-up on all bits to save power ...
    PULLUP_DISABLE(_UART_RX);   // ... except no pull-up on RXD, TXD
    PULLUP_DISABLE(_UART_TX);   

#ifdef SOFT_1MHZ
	/*
	  Fuses are set for internal 8 MHz RC oscillator, no divide-by-8
	  bootloader operates at 8 MHz, 57600 Baud
	  F_CPU is set to 1'000'000 
      at start of application, enable divide-by-8
	*/
	clock_prescale_set(clock_div_8);
#endif
}


/**
 * @brief Basic things to do in setup().
 * 
 */
void basicSetup()
{
	#ifdef SOFT_1MHZ
	  DEBUG_PRINT("* Soft 1 MHz\r\n");
	#endif

	DEBUG_PRINTF("* Fuses: L=%02X H=%02X E=%02X\r\n",
		boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS),
		boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS),
		boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS)
	);
}


//===========================================================================

#define SENSOR_ID_VCC 		99 		// battery voltage

MyMessage msgVCC( SENSOR_ID_VCC, V_VOLTAGE );


/**
 * @brief present battery voltage sensor to MySensors gateway
 * 
 */
void presentBattery()
{
	delay(10);
	present(SENSOR_ID_VCC, S_MULTIMETER, "VCC [mV]");
}


/**
 * @brief Send MySensors messages with battery level [%] and battery voltage [mV]
 * 
 */
void reportBatteryVoltage()
{
	uint16_t batteryVoltage = AvrBattery::measureVCC();
	delay(10);
	send(msgVCC.set(batteryVoltage));
	uint8_t percent = AvrBattery::calcVCC_Percent(batteryVoltage);
	DEBUG_PRINTF("Bat: %u mV = %d%%\r\n", batteryVoltage, percent);
	delay(10);
	sendBatteryLevel(percent);
}

//-----------------------------------------------------------------------------

//===========================================================================
#pragma region Comms status reporting

#define SENSOR_ID_COMMS		98		///< battery voltage
#define V_TYPE_JSON 		V_VAR1	///< combined report, string like "{R:999,T:999,E:999}"
#define V_TYPE_NTX			V_VAR2	///< number of packets transmitted in reporting period
#define V_TYPE_NERR			V_VAR3	///< number of Tx errors in reporting period
#define V_TYPE_ERATE		V_VAR4	///< error rate in percent


MyMessage msgCommsStatus(SENSOR_ID_COMMS, V_TYPE_NERR);

/**
 * @brief Data structure for collecting comms operating parameters
 * 
 */
struct comms_status_t {
	unsigned nRx, nTx, nErr;
} comms_status;

//---------------------------------------------------------------------------

/**
 * @brief Present comms status 'sensor' to MySensors gateway
 * 
 */
void presentCommsStatus()
{
	present( SENSOR_ID_COMMS, S_CUSTOM, "Comms status (JSON)" );
}

//---------------------------------------------------------------------------

/**
 * @brief Analyse notifications received by `indication()`
 * 
 * @param ind  Notification code, from core/MyIndication.h
 */
void processCommsStatus( const indication_t ind )
{
	switch (ind) {
		case INDICATION_TX:		comms_status.nTx++; break;
		case INDICATION_RX: 	comms_status.nRx++; break;
		case INDICATION_ERR_TX:	comms_status.nErr++; break;
		default: break;
	}
}

//---------------------------------------------------------------------------

/**
 * @brief Send accrued operating statistics, then reset values.
 * Call this once per hour or so
 * 
 */
void reportCommsStatus()
{
	static char json[32];
	comms_status_t cs = comms_status;
	memset( &comms_status, 0, sizeof(comms_status) );
	unsigned erate = cs.nTx ? (100uL * cs.nErr / cs.nTx) : 0;

	//{"R":999,"T":999,"E":999}
	snprintf(json,sizeof(json),"{E:%d,T:%u,R:%d}", cs.nErr, cs.nTx, cs.nRx );
	delay(10);
	send( msgCommsStatus.setType(V_TYPE_JSON).set(json) );
	delay(10);
	send( msgCommsStatus.setType(V_TYPE_NTX).set(cs.nTx) );
	delay(10);
	send( msgCommsStatus.setType(V_TYPE_NERR).set(cs.nErr) );
	delay(10);
	send( msgCommsStatus.setType(V_TYPE_ERATE).set(erate) );
}

#pragma endregion
