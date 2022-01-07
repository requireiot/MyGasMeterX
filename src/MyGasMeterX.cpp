/**
 * @file 		  MyGasMeterX.cpp
 *
 * Project		: Home automation
 * Author		: Bernd Waldmann
 * Created		: 03-Oct-2019
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyGasMeterX.cpp 1323 2022-01-07 10:19:03Z  $
 */

/*
   Copyright (C) 2017,2021 Bernd Waldmann

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

#define MY_INDICATION_HANDLER
#include "mysensors_conf.h"
#include <MySensors.h>

#ifdef REPORT_CLIMATE
	#include <Adafruit_Sensor.h>
	#include <Adafruit_BME280.h>
#endif

// my libraries from https://github.com/requireiot/
#include <stdpins.h>
#include <AvrTimers.h>
#include <Button.h>
#include <AvrBattery.h>
#include <debugstream.h>
#include <debugstream_arduino.h>

// project-specific headers
#include "Basics.h"
#include "LuxMeter.h"
#include "pins.h"

//===========================================================================
#pragma region Constants

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
  // time between battery status reports
  const unsigned long BATTERY_REPORT_INTERVAL = 5 MINUTES;
  // Sleep time between reports (in milliseconds)
  const unsigned long MIN_REPORT_INTERVAL = 60 SECONDS;
  // max time between count reports
  const unsigned long MAX_REPORT_INTERVAL = 2 MINUTES;
  // report climate
  const unsigned long CLIMATE_REPORT_INTERVAL = 60 SECONDS;
  // time between light level reports
  const unsigned long LIGHT_REPORT_INTERVAL   = 2 MINUTES;
#else
  // time between battery status reports
  const unsigned long BATTERY_REPORT_INTERVAL = 12 HOURS;
  // min time between count reports
  const unsigned long MIN_REPORT_INTERVAL = 5 MINUTES;
  // max time between count reports
  const unsigned long MAX_REPORT_INTERVAL = 30 MINUTES;
  // report climate
  const unsigned long CLIMATE_REPORT_INTERVAL = 5 MINUTES;
  // time between light level reports
  const unsigned long LIGHT_REPORT_INTERVAL   = 30 MINUTES; 
#endif

//----- IDs and Messages

#define SENSOR_ID_TEMPERATURE 		41
#define SENSOR_ID_HUMIDITY			51
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

relcnt_t oldPulseCount = 0;			///< used to detect changes
uint32_t countPerHour = 0;				///< accumulates clicks for 1 hour
uint32_t t_last_sent;

uint16_t batteryVoltage = 3300;			// last measured battery voltage in mV

AvrTimer2 timer2;

Button magnet;

bool transportSleeping = false;

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region BME280 handling

#ifdef REPORT_CLIMATE

void operator delete(void * ptr, size_t size) { free(ptr); }

class MyBME280 : public Adafruit_BME280
{
	public:
		using Adafruit_BME280::Adafruit_BME280;
		void takeForcedMeasurementNoWait();
};

//---------------------------------------------------------------------------

void MyBME280::takeForcedMeasurementNoWait() 
{
  if (_measReg.mode == MODE_FORCED) {
    // set to forced mode, i.e. "take next measurement"
    write8(BME280_REGISTER_CONTROL, _measReg.get());
  }
}

//---------------------------------------------------------------------------

/// !=0 if BME sensor found and initialized
bool validBME = 0;
/// true if take measurement command sent/to be sent to sensor
bool requestBME = false;

MyBME280 bme; 

//---------------------------------------------------------------------------

bool init_Climate()
{
	DEBUG_PRINT("Initializing BME ... ");
	if (bme.begin(0x76)) {
		DEBUG_PRINT("BME ok\r\n");
		bme.setSampling(
			Adafruit_BME280::MODE_FORCED,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::SAMPLING_NONE,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::FILTER_OFF
		);
        return true;
	} else {
		DEBUG_PRINT("BME280 error\r\n");
        return false;
	}
}

//---------------------------------------------------------------------------

bool request_Climate()
{
    if (validBME)	
        bme.takeForcedMeasurementNoWait();
    return validBME;
}

//---------------------------------------------------------------------------

bool report_Climate()
{
    if (validBME) {
		float t = bme.readTemperature();
		if (t != NAN) {
			send(msgTemperature.set(t,1));
		}
		float h = bme.readHumidity();
		if (h != NAN) {
			send(msgHumidity.set(h,0));
		}
		DEBUG_PRINTF("T=%.1f  H=%.0f\r\n",t,h);
		return true;
    } else return false;
}

#endif // REPORT_CLIMATE

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region ----- battery stuff

#define SENSOR_ID_VCC 		99 		// battery voltage

MyMessage msgVCC( SENSOR_ID_VCC, V_VOLTAGE );


static inline
void presentBattery()
{
	present(SENSOR_ID_VCC, S_MULTIMETER, "VCC [mV]");
}


/**
 * @brief Send MySensors messages with battery level [%] and batery voltage [mV]
 * 
 */
void reportBatteryVoltage()
{
	uint16_t batteryVoltage = AvrBattery::measureVCC();
	send(msgVCC.set(batteryVoltage));
	uint8_t percent = AvrBattery::calcVCC_Percent(batteryVoltage);
	DEBUG_PRINTF("Bat: %u mV = %d%%\r\n", batteryVoltage, percent);
	sendBatteryLevel(percent);
}

//-----------------------------------------------------------------------------
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
	static bool wasDown = false;
	bool isClosed;

    SET_LOW(MAGNET_RET);
    _NOP(); _NOP(); _NOP();

	isClosed = IS_TRUE(MAGNET);
	SET_PA(MIRROR,isClosed);

    SET_HIGH(MAGNET_RET);

	magnet.tick(isClosed);

	if (wasDown != magnet.isDown) {
		wasDown = !wasDown;
		if (wasDown) {
			pulseCount++;
			if (absValid) absPulseCount++;
		}
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
	}
}

//----------------------------------------------------------------------------

void presentation()
{
	static char rev[] = "$Rev: 1323 $";
	char* p = strchr(rev+6,'$');
	if (p) *p=0;

	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("MyGasMeterX", rev+6);

	// Register all sensors to gw (they will be created as child devices)
	//                                    	 1...5...10...15...20...25 max payload
	//                                    	 |   |    |    |    |    |
	present(SENSOR_ID_GAS, S_GAS,        	"Gas flow&vol" );
    presentBattery();

#ifdef REPORT_LIGHT
    presentLux();
#endif // REPORT_LIGHT

#ifdef REPORT_CLIMATE
	//                                    	 1...5...10...15...20...25 max payload
	present(SENSOR_ID_TEMPERATURE, S_TEMP, 	"Temperature [°C]");
	present(SENSOR_ID_HUMIDITY, S_HUM,		"Humidity [%]");
#endif // REPORT_CLIMATE
}

//----------------------------------------------------------------------------

void receive(const MyMessage &message)
{
	if (message.isAck()) return;
	if (message.type==V_VAR1 && message.sensor==SENSOR_ID_GAS) {
		// received absPulseCount start value from server
		absPulseCount = message.getLong();
		absValid = true;
		DEBUG_PRINTF("Rx abs count %ld\r\n",absPulseCount);
		send(msgAbsCount.set(absPulseCount + pulseCount));
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
#ifdef REPORT_CLIMATE
    PULLUP_DISABLE(_I2C_SCL);   // ... except no pull-up on SDA,SCL
    PULLUP_DISABLE(_I2C_SDA);	
#endif

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
}

//---------------------------------------------------------------------------
#pragma endregion
//===========================================================================
#pragma region Arduino framework functions

void setup()
{
    basicSetup();

	// when entering setup(), a lot of RF packets have just been transmitted, so
	// let's wait a bit to let the battery voltage recover, then report
	sleep(100);
	reportBatteryVoltage();

	// Fetch last known pulse count value from gw
	request(SENSOR_ID_GAS, V_VAR1);
	send(msgAbsCount.set(0));	// this triggers sending the "real" value

	timer2.begin(ISR_RATE, 0, myISR, 32768ul, true);		// async mode, 32768 Hz clock
    timer2.handle_millis();
    TIMSK0 = 0;							// disable all T0 interrupts (Arduino millis() )
	timer2.start();		// start debouncing the switch

    t_last_sent = timer2.get_millis();

#ifdef REPORT_CLIMATE
	validBME = init_Climate();
#endif // REPORT_CLIMATE

	//           1...5...10........20........30........40        50        60  63
	//           |   |    |    |    |    |    |    |    |    |    |    |    |   |
	//                                                            23:59:01"
	DEBUG_PRINT("$Id: MyGasMeterX.cpp 1323 2022-01-07 10:19:03Z  $ " __TIME__ "\r\n" ) ;
    DEBUG_PRINTF("Node: %d\r\n", MY_NODE_ID);
	Serial.flush();
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

	bool maySendNow = ((uint32_t)(t_now - t_last_sent) >= MIN_REPORT_INTERVAL );
	bool mustSendNow = ((uint32_t)(t_now - t_last_sent) >= MAX_REPORT_INTERVAL );

	if (maySendNow) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			_relCount = pulseCount;
			if (absValid) pulseCount=0;
			_absCount = absPulseCount;
		}
		if (_relCount || mustSendNow || !absValid) {
			if (absValid) {
				send(msgRelCount.set(_relCount));
				send(msgAbsCount.set(_absCount));
			} else {
				// only send relative counts
				send(msgRelCount.set(_relCount));
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
	if ( (unsigned long)(t_now - t_hourly) > 1 HOURS ) {
		t_hourly = t_now;
		uint32_t liters;
		liters = countPerHour * LITERS_PER_CLICK;
		send(msgGasFlow.set(liters));
		if (absValid) {
			liters = absPulseCount * LITERS_PER_CLICK;
			send(msgGasVolume.set(liters));
		}
		transportSleeping = false;
		countPerHour = 0;
	}

#ifdef REPORT_LIGHT
	static uint32_t t_light_report=0uL;

	// every 30min or so, report light
	if ((unsigned long)(t_now - t_light_report) >= LIGHT_REPORT_INTERVAL) {
		t_light_report = t_now;
        reportLux();
		transportSleeping = false;
	}
#endif 

	// once a day or so, report battery status
  	if ((unsigned long)(t_now - t_battery_report) >= BATTERY_REPORT_INTERVAL) {
		t_battery_report = t_now;
		reportBatteryVoltage();
		transportSleeping = false;
	}

#ifdef REPORT_CLIMATE
	static uint32_t t_climate_report=0uL;
	//static uint32_t t_climate_read;

	// if BME sensor measurement was triggered before last sleep period, report it now
	if (validBME && requestBME && ((unsigned long)(t_now - t_climate_report) > 10uL ))	{
        report_Climate();
		requestBME = false;
		transportSleeping = false;
	}

  	if ((unsigned long)(t_now - t_climate_report) >= CLIMATE_REPORT_INTERVAL) {
		t_climate_report = t_now;
		requestBME = request_Climate();		// trigger BME280 measurement before next snooze
	}
#endif // REPORT_CLIMATE
}

//---------------------------------------------------------------------------
#pragma endregion
