/**
 * @file 		  MyGasMeterX.cpp
 *
 * Project		: Home automation
 * Author		: Bernd Waldmann
 * Created		: 03-Oct-2019
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyGasMeterX.cpp 1451 2022-10-27 14:58:38Z  $
 */

/*
   Copyright (C) 2017,2022 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/*
 * Relies on MySensors, Created by Henrik Ekblad <henrik.ekblad@mysensors.org>.
 * Note that the MySensors library is under GPL, so 
 * - if you want to combine this source code file with the MySensors library 
 *   and redistribute it, then read the GPL to find out what is allowed.
 * - if you want to combine this file with the MySensors library and only 
 *   use it at home, then read the GPL to find out what is allowed.
 * - if you want to take just this source code, learn from it, modify it, and 
 *   redistribute it, independent from MySensors, then you have to abide by 
 *   my license rules (MPL 2.0), which imho are less demanding than the GPL.
 */

/**
	 @brief Gas meter via magnetic sensor, optional BME280 climate sensor, brightness sensor.

	For low power operation, this module uses a 32768 Hz watch crystal to create 
	interrupts every 10ms, and spends as much time in SLEEP_MODE_PWR_SAVE sleep
	as possible.

	Initially, the gas meter only report **incremental** data:
	- frequently, we report the incremental pulse count since the last report 
	  (msgRelCount)
	- once per hour, we report flow [liters/hour] calculated from pulse count 
	  (msgGasFlow)

	Once we have received from controller a **base count** value
	(message SENSOR_ID_GAS / V_VAR1), we also start reporting absolute data:
	- frequently, we report the absolute pulse count, 
	  i.e. base value + pulses since boot (msgAbsCount)
	- once per hour, we report total gas volume [liters] consumed,
	  i.e. (base value + pulses since boot) * liters/count
	  (msgGasVolume)

	To set the **base count** value via MQTT, 
	- say the sensor is node #126, then in one shell listen to messages from that node
	  `mosquitto_sub -t 'my/+/stat/126/#'`
	- wait for the sensor to send its initial report, e.g.
      `my/2/stat/126/81/1/0/24 0`
	  `my/2/stat/126/81/2/0/24`
	- then, in another shell, set the initial value (say gas meter showed 6591,970 m³)
	  `mosquitto_pub -t "my/cmnd/126/81/1/0/24" -m '659197'`
	(in my setup, the MySensors gateway publishes messages from MySensors nodes as `my/2/stat/#`,
	and it subscribes to `my/cmnd/#` messages to a node )
*/

// standard hearders
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdbool.h>

// Arduino and 3rd party libraries
#include <Arduino.h>
#include <Wire.h>

#ifdef USE_BME280_MIN
	#include <LeanBME280.h>
#endif

#define MY_BAUD_RATE 9600uL

#define MY_SPLASH_SCREEN_DISABLED
#define MY_INDICATION_HANDLER
#define MY_RADIO_RF24
#include "mysensors_conf.h"
#include <MySensors.h>

// my libraries from https://github.com/requireiot/
#include <stdpins.h>
#include <AvrTimers.h>
#include <Button.h>
#include <AvrBattery.h>
#include <MyBasicNode.h>
#include <debugstream.h>
#include <debugstream_arduino.h>
#include <ansi.h>

// project-specific headers
#include "LuxMeter.h"
#include "pins.h"

//===========================================================================
#pragma region Constants

#if defined(USE_BME280_MIN)
 #define REPORT_CLIMATE
#endif


#define ISR_RATE 	100		// interrupt rate in Hz
#define LOOP_RATE	1		// rate of executing loop(), in Hz

#define LITERS_PER_CLICK 10		// depends on gas meter, this is for G4 Metrix 6G4L

//----- timing

#define SECONDS		* 1000uL
#define MINUTES 	* 60uL SECONDS
#define HOURS 		* 60uL MINUTES
#define DAYS		* 24uL HOURS

// #define QUICK   // for debugging

#ifdef QUICK
  // Sleep time between reports (in milliseconds)
  const unsigned long MIN_REPORT_INTERVAL = 60 SECONDS;
  // max time between count reports
  const unsigned long MAX_REPORT_INTERVAL = 2 MINUTES;
  // time between comms status reports
  const unsigned long COMMS_REPORT_INTERVAL = 1 MINUTES;
  // time between battery status reports
  const unsigned long BATTERY_REPORT_INTERVAL = 5 MINUTES;
  // report climate
  const unsigned long CLIMATE_REPORT_INTERVAL = 60 SECONDS;
  // time between light level reports
  const unsigned long LIGHT_REPORT_INTERVAL   = 2 MINUTES;
#else
  // min time between count reports
  const unsigned long MIN_REPORT_INTERVAL = 5 MINUTES;
  // max time between count reports
  const unsigned long MAX_REPORT_INTERVAL = 30 MINUTES;
  // time between comms status reports
  const unsigned long COMMS_REPORT_INTERVAL = 2 HOURS;
  // time between battery status reports
  const unsigned long BATTERY_REPORT_INTERVAL = 12 HOURS;
  // report climate
  const unsigned long MIN_CLIMATE_REPORT_INTERVAL = 1 MINUTES;
  const unsigned long MAX_CLIMATE_REPORT_INTERVAL = 5 MINUTES;
  // time between light level reports
  const unsigned long LIGHT_REPORT_INTERVAL   = 30 MINUTES; 
#endif

//----- IDs and Messages

#define SENSOR_ID_TEMPERATURE 		41
#define SENSOR_ID_HUMIDITY			51
#define SENSOR_ID_BARO				52
#define SENSOR_ID_GAS				81   	// gas volume in clicks and m3/h

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region Global variables

MyMessage msgGasFlow(SENSOR_ID_GAS, V_FLOW);		// in l/h			my/+/stat/120/81/1/0/34
MyMessage msgGasVolume(SENSOR_ID_GAS, V_VOLUME);	// l accumulated	my/+/stat/120/81/1/0/35
MyMessage msgAbsCount(SENSOR_ID_GAS,V_VAR1);		// absolute clicks  my/+/stat/120/81/1/0/24 or my/cmnd/120/81/1/0/24
MyMessage msgRelCount(SENSOR_ID_GAS,V_VAR2);		// clicks since last report  my/+/stat/120/81/1/0/25

#ifdef REPORT_CLIMATE
	MyMessage msgTemperature(SENSOR_ID_TEMPERATURE, V_TEMP);
	MyMessage msgHumidity(SENSOR_ID_HUMIDITY, V_HUM);
	MyMessage msgBaro(SENSOR_ID_BARO, V_PRESSURE);
#endif

/*
	annual consumption is ca 1'000 m3, or 1'000'000 liters
	uint32_t good enough for 2000 years ...

	max observed is 20 counts/5min. 
	Let's plan for 10/min, or 50/5min ... uint8_t good enough for pulseCount
	That's 600/h, so uint16_t good enough for countPerHour
*/

typedef uint8_t relcnt_t;

bool absValid = false;					///< true if initial value has been received from gateway

volatile relcnt_t pulseCount = 0;		///< counter for magnet pulses (clicks), updated in ISR
volatile uint32_t absPulseCount = 0;	///< cumulative pulse count

relcnt_t oldPulseCount = 0;				///< used to detect changes
uint32_t countPerHour = 0;				///< accumulates clicks for 1 hour
uint32_t t_last_sent;

uint16_t batteryVoltage = 3300;			// last measured battery voltage in mV

AvrTimer2 timer2;

//Button magnet;
Contact magnet;

bool transportSleeping = false;

float oldT=NAN, oldH=NAN, oldP=NAN;
unsigned hasClimateSensor = 0;

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region BME280 handling (Lean library)

#ifdef USE_BME280_MIN

LeanBME280 bme(0x76);


/**
 * @brief Read temperature and humidity from sensor.
 * This assumes that (a) there is a valid BME280 and 
 * (b) takeForcedMeasurementNoWait() has been called a while ago.
 * 
 * This takes 7ms for T+H only, or 11ms for T,H,P, when waiting for measurement
 * This takes 1ms (T,H) or 2,5ms (T,H,P) when interleaved
 */
void readClimate( float& t, float& h, float& p )
{
	int16_t it,ih,ip;

#ifdef USE_BARO
	bool ok = bme.readSensor( &it, &ih, &ip );
#else
	bool ok = bme.readSensor( &it, &ih );
#endif
	if (ok) {
		t = it / 100.0;
		h = ih;
#ifdef USE_BARO
		p = ip;
#endif
	} else {
		t = NAN;
		h = NAN;
		p = NAN;
	}

	// trigger next measurement
	bme.takeForcedMeasurementNoWait();
}


bool initClimate()
{
	// with Minimal BME280 library and 8 MHz F_CPU, wake time is ca x ms
	DEBUG_PRINT("Initializing BME280 (Lean) ... ");
	
	bool ok = bme.begin();
	if (ok) {
		DEBUG_PRINT("ok\r\n");
		bme.setSampling(
			BME280_MODE_SLEEP,
			BME280_SAMPLING_X1,
#ifdef USE_BARO
			BME280_SAMPLING_X1,
#else
			BME280_SAMPLING_NONE,
#endif
			BME280_SAMPLING_X1
		);
		bme.takeForcedMeasurement();
		readClimate( oldT, oldH, oldP );
#ifdef USE_BARO
		DEBUG_PRINTF( ANSI_BOLD "%.0f" ANSI_RESET " hPa  ", oldP);
#endif
		DEBUG_PRINTF( ANSI_BOLD "%.1f" ANSI_RESET " °C  ", oldT);
		DEBUG_PRINTF( ANSI_BOLD "%.0f" ANSI_RESET " %%rH\r\n", oldH);
	} else {
		DEBUG_PRINT("error\r\n");
	}
	return ok;
}

#endif // USE_BME280_MIN

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region Local functions

/*
    ISR is called every 10ms, debouncer needs 4 samples to recognize edge, 
    so min 40ms = 25 Hz pulse rate. In reality, meter does > 5s/pulse
*/

/**
 * @brief called periodically by Timer2 ISR
 */
void myISR(void) 
{
	//static bool wasDown = false;
	bool isClosed;

    SET_LOW(MAGNET_RET);
	_delay_us(10);
	isClosed = IS_TRUE(MAGNET);
	SET_PA(MIRROR,isClosed);
    SET_HIGH(MAGNET_RET);

	magnet.tick(isClosed);
	if (magnet.cPressed) {
		magnet.cPressed = 0;
		pulseCount++;
		if (absValid) 
			absPulseCount++;
	}
}


/**
 * @brief sleep until the next time loop() needs to run.
 * 
 * @param allowTransportDisable  if True, turn off NRF24
 * 
 * The reporting functions in loop() only need to run every 1s, 
 * so if the Timer2 interrupt is more frequent, to enable the 
 * debouncing routine, then return to loop() only once every 1s.
 * 
 * Short version of a wake period (only poll contact) takes ~630ns @ 8 MHz
 * Long version of a wake period (run loop()) takes ~75µs @ 8 MHz (longer if RF transmission). 
 */
void snooze(bool allowTransportDisable)
{
	while (!isTransportReady()) { _process(); }
	if (allowTransportDisable && !transportSleeping) {
		transportDisable();
		transportSleeping = true;
	}
	Serial.flush();

	for (uint8_t t=0; t<(ISR_RATE/LOOP_RATE); t++) {
		indication(INDICATION_SLEEP);
		set_sleep_mode(SLEEP_MODE_PWR_SAVE);	
		cli();
		sleep_enable();
#if defined __AVR_ATmega328P__
		sleep_bod_disable();		
#endif
		sei();
		sleep_cpu();
		sleep_disable();
		indication(INDICATION_WAKEUP);
	}
}

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region light sensor

#ifdef REPORT_LIGHT

#define SENSOR_ID_LIGHT	 			61		// light sensor in %

MyMessage msgLux( SENSOR_ID_LIGHT, V_LIGHT_LEVEL );


static inline
void presentLux()
{
	// Register sensors to gw
	//                                    	 1...5...10...15...20...25 max payload
	//                                    	 |   |    |    |    |    |
	present(SENSOR_ID_LIGHT, S_LIGHT_LEVEL, "Light [%]");
}


static inline
void reportLux()
{
    uint16_t u = measureLux();
    send(msgLux.set(u));

}

#endif // REPORT_LIGHT

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region MySensor framework functions

void indication( const indication_t ind )
{
	if (ind==INDICATION_SLEEP) {
		NEGATE(AWAKE);
	} else if (ind==INDICATION_WAKEUP) {
		ASSERT(AWAKE);
	} else processCommsStatus(ind);
}

//----------------------------------------------------------------------------

void presentation()
{
	static char rev[] = "$Rev: 1451 $";
	char* p = strchr(rev+6,'$');
	if (p) *p=0;

	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("MyGasMeterX"
#ifdef USE_BME280_MIN
		" (280min)"
#endif
	, rev+6);

	// Register all sensors to gw (they will be created as child devices)
	//                                    	 1...5...10...15...20...25 max payload
	//                                    	 |   |    |    |    |    |
	delay(10);	// pause between messages, Linux gateways are slow
	present(SENSOR_ID_GAS, S_GAS,        	"Gas flow&vol" );

#ifdef REPORT_LIGHT
	delay(10);	// pause between messages, Linux gateways are slow
    presentLux();
#endif // REPORT_LIGHT

#ifdef REPORT_CLIMATE
	//                                    	 1...5...10...15...20...25 max payload
	delay(2);	// pause between messages, Linux gateways are slow
	present(SENSOR_ID_TEMPERATURE, S_TEMP, 	"Temperature [°C]");
	delay(2);	// pause between messages, Linux gateways are slow
	present(SENSOR_ID_HUMIDITY, S_HUM,		"Humidity [%]");
 #ifdef USE_BARO
	delay(2);
	present( SENSOR_ID_BARO, 		S_BARO, F("Baro pressure [hPa]"));
 #endif	
#endif // REPORT_CLIMATE

    presentBattery();
    presentCommsStatus();
	presentArcStatistics();
}

//----------------------------------------------------------------------------

void receive(const MyMessage &message)
{
	if (message.isAck()) return;
	if (message.type==V_VAR1 && message.sensor==SENSOR_ID_GAS) {
		// received absPulseCount start value from server
		uint32_t _absCount = message.getLong();
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			absPulseCount = _absCount;
			absPulseCount += pulseCount;
			_absCount = absPulseCount;
		}
		absValid = true;
		DEBUG_PRINTF("Rx abs count %ld\r\n",absPulseCount);
		send(msgAbsCount.set(_absCount));
	}
}

//---------------------------------------------------------------------------

/**
 * @brief Initialize hardware pins. 
 * Called early in the boot sequence by MySensors framework
 * 
 */
void preHwInit()
{
    basicHwInit();

    // configure pins used by this application

    AS_OUTPUT(AWAKE);
    ASSERT(AWAKE);

    AS_OUTPUT(MIRROR);
    NEGATE(MIRROR);

	AS_OUTPUT(MAGNET_RET);
	SET_HIGH(MAGNET_RET);
    
	AS_INPUT_PU(MAGNET);	

#ifdef REPORT_LIGHT	
    initLux();
#endif // REPORT_LIGHT

#ifdef REPORT_CLIMATE
	// no pull-up on SDA,SCL
	AS_INPUT_FLOAT(_I2C_SDA);
	AS_INPUT_FLOAT(_I2C_SCL);
	PRR &= ~_BV(PRTWI);
#endif
}

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region Arduino framework functions

void reportClimate(float t, float h, float p)
{
	DEBUG_PRINTF("%.1f °C  ",t);
	if (!isnan(t)) {
		my_send(msgTemperature.set(t,1));
	} else sendDebugString("T=NaN");
	DEBUG_PRINTF("%.0f %%rH  ",h);
	if (!isnan(h)) { 
		delay(2);
		my_send(msgHumidity.set(h,0));
	}
	DEBUG_PRINTF("%.0f hPa",p);
	if (!isnan(p)) { 
		delay(2);
		my_send(msgBaro.set(p,0));
	}
	DEBUG_PRINT("\r\n");
}


void setup()
{
    basicSetup();

	// when entering setup(), a lot of RF packets have just been transmitted, so
	// let's wait a bit to let the battery voltage recover, then report
	sleep(100);
	reportBatteryVoltage();

	// Fetch last known pulse count value from gw
	request(SENSOR_ID_GAS, V_VAR1);
	my_send(msgAbsCount.set(0));	// this triggers sending the "real" value

	timer2.begin(ISR_RATE, 0, myISR, 32768ul, true);		// async mode, 32768 Hz clock
    timer2.handle_millis();
    TIMSK0 = 0;							// disable all T0 interrupts (Arduino millis() )
	timer2.start();		// start debouncing the switch

    t_last_sent = timer2.get_millis();

#ifdef REPORT_CLIMATE
	hasClimateSensor = initClimate();
	reportClimate( oldT, oldH, oldP );
#endif

	//           1...5...10........20........30........40        50        60  63
	//           |   |    |    |    |    |    |    |    |    |    |    |    |   |
	//                                                            23:59:01"
	DEBUG_PRINT("$Id: MyGasMeterX.cpp 1451 2022-10-27 14:58:38Z  $ " __TIME__ "\r\n" ) ;
    DEBUG_PRINTF("Node: %d\r\n", MY_NODE_ID);
	Serial.flush();

	sendDebugMessage(F("parent: %u"), transportGetParentNodeId());
	reportArcStatistics();	
}

//----------------------------------------------------------------------------

void loop()
{
	static uint32_t t_battery_report=0uL;
	static uint32_t t_hourly = 0;
	relcnt_t _relCount;
	uint32_t _absCount;

	snooze(absValid);
	
	uint32_t t_now = timer2.get_millis();

	bool maySendNow  = ((uint32_t)(t_now - t_last_sent) >= MIN_REPORT_INTERVAL );
	bool mustSendNow = ((uint32_t)(t_now - t_last_sent) >= MAX_REPORT_INTERVAL );

	if (maySendNow) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			_relCount = pulseCount;
			if (absValid) pulseCount=0;
			_absCount = absPulseCount;
		}
		if (_relCount || mustSendNow || !absValid) {
			my_send(msgRelCount.set(_relCount));
			if (absValid) {
				// we have a valid baseline, so send total and relative counts
				delay(2);	// pause between messages, Linux gateways are slow
				my_send(msgAbsCount.set(_absCount));
			} else {
				// only send relative counts
				DEBUG_PRINT("Requesting AbsCount\r\n");
				request(SENSOR_ID_GAS, V_VAR1);
			}
			transportSleeping = false;
			countPerHour += _relCount;
			DEBUG_PRINTF("rel %ld, abs %ld\r\n", _relCount, _absCount);
			t_last_sent = t_now;
		}
	}

	// once per hour, calculate and report liters/h
	if ( (uint32_t)(t_now - t_hourly) > 1 HOURS ) {
		t_hourly = t_now;
		uint32_t liters;
		liters = countPerHour * LITERS_PER_CLICK;
		my_send(msgGasFlow.set(liters));
		if (absValid) {
			liters = absPulseCount * LITERS_PER_CLICK;
			delay(2);	// pause between messages, Linux gateways are slow
			my_send(msgGasVolume.set(liters));
		}
		transportSleeping = false;
		countPerHour = 0;
	}

#ifdef REPORT_LIGHT
	static uint32_t t_light_report=0uL;

	// every 30min or so, report light
	if ((uint32_t)(t_now - t_light_report) >= LIGHT_REPORT_INTERVAL) {
		t_light_report = t_now;
        reportLux();
		transportSleeping = false;
	}
#endif 

	// once a day or so, report comms statistics
	static uint32_t t_comms_report=0uL;
  	if ((unsigned long)(t_now - t_comms_report) >= COMMS_REPORT_INTERVAL) {
		t_comms_report = t_now;
		reportCommsStatus();
		reportArcStatistics();
		transportSleeping = false;
	}	

#ifdef REPORT_CLIMATE
	static uint32_t t_climate_report=0uL;
	// wake time without RF transmission is  BME280/Erriez: 2ms
	if (hasClimateSensor) {
		float newT=NAN, newH=NAN, newP=NAN;
		readClimate(newT,newH, newP);
#ifdef QUICK
 #ifdef USE_BARO
		DEBUG_PRINTF( ANSI_BOLD "%.0f" ANSI_RESET " hPa  ", newP );
 #endif
		DEBUG_PRINTF( ANSI_BOLD "%.1f" ANSI_RESET " °C  ", newT );
		DEBUG_PRINTF( ANSI_BOLD "%.0f" ANSI_RESET " %%rH\r\n", newH);
#endif // QUICK
		if ( (unsigned long)(t_now - t_climate_report) >= MIN_CLIMATE_REPORT_INTERVAL )	// report no more than 1/min
		{
			if (												// do we need to report now?		
				( !isnan(newT) && fabsf(newT-oldT) >= 1.0) ||	// report if temperature changed by >1°
				( !isnan(newT) && isnan(oldT) ) ||				// report if previous value was invalid
				( !isnan(newH) && fabsf(newH-oldH) >= 2.0) ||	// report if humidity changed by >2%
				( !isnan(newH) && isnan(oldH) ) ||				// report if previous value was invalid
				( !isnan(newP) && fabsf(newP-oldP) >= 10.0) ||	// report if pressure changed by > 10 mbar
				( !isnan(newP) && isnan(oldP) ) ||				// report if previous value was invalid
				(unsigned long)(t_now - t_climate_report) >= MAX_CLIMATE_REPORT_INTERVAL	// report at least every 15 min
			) {
				t_climate_report = t_now;
				oldT = newT;
				oldH = newH;
				oldP = newP;
				reportClimate( newT, newH, newP );
				transportSleeping = false;
			}
		}
	}
#endif // REPORT_CLIMATE

	// once a day or so, report battery status
  	if ((uint32_t)(t_now - t_battery_report) >= BATTERY_REPORT_INTERVAL) {
		t_battery_report = t_now;
		reportBatteryVoltage();
		transportSleeping = false;
	}

}

//---------------------------------------------------------------------------
#pragma endregion
