/**
 * @file LeanBME280.cpp
 * @author Bernd Waldmann
 * @date 2022-02-26
 *
 * This Revision: $Id: $
 */

/*
   Copyright (C) 20221 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License,
   v. 2.0. If a copy of the MPL was not distributed with this file, You can
   obtain one at http://mozilla.org/MPL/2.0/ .

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief BME280 sensor library for Arduino, optimized for speed and footprint.
 *
 * optimized for memory: avoid all floating-point operations.
 * optimized for speed: avoid repetitive calculations (temperature compensation)
 * detects I2C communications errors and can report them to the application.
 *
 * inspired by Erriez BME/BMP280 library https://github.com/Erriez/ErriezBMX280
 * (MIT license),
 * inspired by Adafruit library https://github.com/adafruit/Adafruit_BME280_Library,
 * but major changes to logic and API.
 *
 * includes compensation formulas from Bosch Sensortec code at
 * https://github.com/BoschSensortec/BME280_driver (BSD-3-Clause license).
 * Those code snippets are Copyright (c) 2020 Bosch Sensortec GmbH, I suppose.
 */

#include "LeanBME280.h"

//#define BME_LOGGING

#ifdef BME_LOGGING
 #include <debugstream.h>
 #define BME_LOG_PRINTF DEBUG_PRINTF
 #define BME_LOG_PRINT DEBUG_PRINT
#else
 #define BME_LOG_PRINTF(...)
 #define BME_LOG_PRINT(s)
#endif

//=============================================================================
#pragma region BME280 register definitions

// Bit defines
#define CHIP_ID_BMP280 0x58 ///< BMP280 chip ID
#define CHIP_ID_BME280 0x60 ///< BME280 chip ID
#define RESET_KEY 0xB6      ///< Reset value for reset register
#define STATUS_IM_UPDATE 0  ///< im_update bit in status register

// BME280 registers
#define BME_REG_DIG_T1 0x88 ///< Temperature coefficient register
#define BME_REG_DIG_T2 0x8A ///< Temperature coefficient register
#define BME_REG_DIG_T3 0x8C ///< Temperature coefficient register

#define BME_REG_DIG_P1 0x8E ///< Pressure coefficient register
#define BME_REG_DIG_P2 0x90 ///< Pressure coefficient register
#define BME_REG_DIG_P3 0x92 ///< Pressure coefficient register
#define BME_REG_DIG_P4 0x94 ///< Pressure coefficient register
#define BME_REG_DIG_P5 0x96 ///< Pressure coefficient register
#define BME_REG_DIG_P6 0x98 ///< Pressure coefficient register
#define BME_REG_DIG_P7 0x9A ///< Pressure coefficient register
#define BME_REG_DIG_P8 0x9C ///< Pressure coefficient register
#define BME_REG_DIG_P9 0x9E ///< Pressure coefficient register

#define BME_REG_DIG_H1 0xA1 ///< Humidity coefficient register
#define BME_REG_DIG_H2 0xE1 ///< Humidity coefficient register
#define BME_REG_DIG_H3 0xE3 ///< Humidity coefficient register
#define BME_REG_DIG_H4 0xE4 ///< Humidity coefficient register
#define BME_REG_DIG_H5 0xE5 ///< Humidity coefficient register
#define BME_REG_DIG_H6 0xE7 ///< Humidity coefficient register

#define BME_REG_CHIPID 0xD0 ///< Chip ID register
#define BME_REG_RESET 0xE0  ///< Reset register

#define BME_REG_CTRL_HUM 0xF2  ///< BME280: Control humidity register
#define BME_REG_STATUS 0xF3    ///< Status register
#define BME_REG_CTRL_MEAS 0xF4 ///< Control measure register
#define BME_REG_CONFIG 0xF5    ///< Config register
#define BME_REG_PRESS 0xF7     ///< Pressure data register
#define BME_REG_TEMP 0xFA      ///< Temperature data register
#define BME_REG_HUM 0xFD       ///< Humidity data register

#pragma endregion
//=============================================================================
#pragma region High - level functions

/**
 * @brief Constructor
 * @param i2cAddr   I2C address 0x76 or 0x77
 */
LeanBME280::LeanBME280(uint8_t i2cAddr) : _i2cAddr(i2cAddr), _t_fine(0)
{
}

/**
 * @brief Initialize BME280 sensor.
 * @retval true if BME280 sensor detected
 */
bool LeanBME280::begin()
{
    Wire.setClock(100000uL);
	//Wire.setWireTimeout();

    // Read chip ID
    _chipID = read8(BME_REG_CHIPID);

    // Check sensor ID BMP280 or BME280
    if (_chipID != CHIP_ID_BME280)
    {
        // BMP280 / BME280 not found
        return false;
    }

    // Generate soft-reset
    write8(BME_REG_RESET, RESET_KEY);

    // Wait for copy completion NVM data to image registers
    delay(10);
    while (read8(BME_REG_STATUS) & _BV(STATUS_IM_UPDATE))
    {
        delay(1);
    }

    // See datasheet 4.2.2 Trimming parameter readout
    if (!readCoefficients()) return false;

    // Set default sampling
    setSampling();

    return true;
}

/**
 * @brief Trigger single measurement, don't wait for completion.
 *
 */
void LeanBME280::takeForcedMeasurementNoWait()
{
    // uint8_t reg = read8(BME_REG_CTRL_MEAS) & 0b11111100;
    write8(BME_REG_CTRL_MEAS, _ctrlmeas | BME280_MODE_FORCED);
}

/**
 * @brief Trigger single measurement and wait for completion.
 *
 */
void LeanBME280::takeForcedMeasurement()
{
    takeForcedMeasurementNoWait();
    while (!(read8(BME_REG_STATUS) & _BV(3)))
    {
        if (_wireError) return;
        delay(1);
    }
    BME_LOG_PRINT(" ! ");
    while (read8(BME_REG_STATUS) & _BV(3))
    {
        if (_wireError) return;
        delay(10);
    }
}

/**
 * @brief Read some or all climate parameters from sensor
 *
 * @param pTemp     pointer to temperature (1/100°C) variable or NULL for 'not interested'
 * @param pHum      pointer to humidity (%rH) variable or NULL for 'not interested'
 * @param pBaro     pointer to pressure (hPa) variable or NULL for 'not interested'
 * @return
 *      true if all requested parameters could be read,
 *      false if anything went wrong with I2C communication
 */
bool LeanBME280::readSensor(int16_t *pTemp, int16_t *pHum, int16_t *pBaro)
{
    int16_t t = readTemperature();
    if (_wireError) return false;
    if (t==INAN) return false;

    if (pTemp) 
        *pTemp = t;
    if (pHum) 
        *pHum = _readHumidity();
    if (pBaro) 
        *pBaro = _readPressure();
    return true;
}

/**
 * @brief Read last temperature measurement and apply compensation.
 * @return  Temperature in 1/100 °C  or INAN if error
 *
 * Compensation formulas from https://github.com/BoschSensortec/BME280_driver/
 */
int16_t LeanBME280::readTemperature()
{
    int32_t var1, var2, temperature;

    if (!readMeasurements()) return INAN;
    if (_rawT == 0x80000L) return INAN;

    var1 = (int32_t)((_rawT >> 3) - ((int32_t)_dig_T1 * 2));
    var1 = (var1 * ((int32_t)_dig_T2)) >> 11;
    var2 = (int32_t)((_rawT >> 4) - ((int32_t)_dig_T1));
    var2 = (((var2 * var2) >> 12) * ((int32_t)_dig_T3)) >> 14;
    _t_fine = var1 + var2;
    temperature = (_t_fine * 5L + 128L) >> 8;
    temperature = constrain(temperature, -4000, 8500);
    BME_LOG_PRINTF(" T=%ld ", temperature);
    return temperature;
}

/**
 * @brief   Read last pressure measurement and apply compensation.
 * @return  Pressure in hPa or INAN if error
 *
 */
int16_t LeanBME280::readPressure()
{
    // Read temperature for t_fine
    if (INAN == readTemperature()) return INAN;
    return _readPressure();
}

/**
 * @brief   Read last pressure measurement and apply compensation.
 * Assumes that temperature has recently been calculated.
 * @return  Pressure in hPa or INAN if error
 *
 * Compensation formulas from https://github.com/BoschSensortec/BME280_driver/
 */
int16_t LeanBME280::_readPressure()
{
    int32_t var1, var2, var3, var4;
    uint32_t var5, pressure;

    var1 = (((int32_t)_t_fine) >> 1) - (int32_t)64000L;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)_dig_P6);
    var2 = var2 + ((var1 * ((int32_t)_dig_P5)) * 2);
    var2 = (var2 >> 2) + (((int32_t)_dig_P4) << 16);
    var3 = (_dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3;
    var4 = (((int32_t)_dig_P2) * var1) >> 1;
    var1 = (var3 + var4) >> 18;
    var1 = (((32768L + var1)) * ((int32_t)_dig_P1)) >> 15;

    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576L) - _rawP;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 >> 12))) * 3125;

        if (pressure < 0x80000000L)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)_dig_P9) * ((int32_t)(((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(pressure >> 2)) * ((int32_t)_dig_P8)) >> 13;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + _dig_P7) >> 4));
        pressure = constrain(pressure, 30000uL, 110000uL);
    }
    else
    {
        pressure = 30000uL;
    }

    BME_LOG_PRINTF(" P'=%ld ", pressure);
    return pressure / 100L;
}

/**
 * @brief Read last humidity measurement and apply compensation.
 * @return  Humidity in %rH or INAN if error
 */
int16_t LeanBME280::readHumidity()
{
    if (INAN == readTemperature()) return INAN; // Read temperature into _t_fine
    return _readHumidity();
}

/**
 * @brief Read last humidity measurement and apply compensation.
 * Assumes that temperature has recently been calculated.
 * @return  Humidity in %rH or INAN if error
 *
 * Compensation formulas from https://github.com/BoschSensortec/BME280_driver/
 */
int16_t LeanBME280::_readHumidity()
{
    int32_t var1, var2, var3, var4, var5;
    int32_t humidity;

    if (_wireError) return INAN;
    if (_rawH == 0x8000) return INAN;

    var1 = _t_fine - ((int32_t)76800L);
    var2 = (int32_t)(_rawH << 14L);
    var3 = (int32_t)(((int32_t)_dig_H4) * 1048576L);
    var4 = ((int32_t)_dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) >> 15;
    var2 = (var1 * ((int32_t)_dig_H6)) >> 10;
    var3 = (var1 * ((int32_t)_dig_H3)) >> 11;
    var4 = ((var2 * (var3 + (int32_t)32768L)) >> 10) + (int32_t)2097152L;
    var2 = ((var4 * ((int32_t)_dig_H2)) + 8192) >> 14;
    var3 = var5 * var2;
    var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
    var5 = var3 - ((var4 * ((int32_t)_dig_H1)) >> 4);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400L ? 419430400L : var5);
    humidity = (uint32_t)(var5 >> 12);
    BME_LOG_PRINTF(" H=%ld ", humidity);
    return humidity >> 10;
}

/**
 * @brief Read coefficient registers at startup.
 */
bool LeanBME280::readCoefficients()
{
    uint8_t ca[26];
    uint8_t cb[16];
    uint8_t i;

    BME_LOG_PRINT(" rCa:");
    for (i=0; i<sizeof(ca); i++) {
        uint8_t b = read8(0x88+i);
        if (_wireError) {
            BME_LOG_PRINT(" errR ");
            return false;
        }
        ca[i] = b;
    }
    /*
    if (sizeof(ca) != Wire.requestFrom(_i2cAddr, sizeof(ca), 0x88, 1, true))
    {
        _wireError = 4;
        BME_LOG_PRINT(" errR ");
        return false;
    }
    for (i = 0; i < sizeof(ca); i++)
        ca[i] = Wire.read();
    */
    BME_LOG_PRINT("ok ");

    BME_LOG_PRINT(" rCb:");
    if (sizeof(cb) != Wire.requestFrom(_i2cAddr, sizeof(cb), 0xE1, 1, true))
    {
        _wireError = 4;
        BME_LOG_PRINT(" errR ");
        return false;
    }
    for (i = 0; i < sizeof(cb); i++)
        cb[i] = Wire.read();
    BME_LOG_PRINT("ok ");

#define A_U16_LE(x) (uint16_t)((uint16_t)ca[x - 0x88 + 1] << 8 | ca[x - 0x88])
#define A_S16_LE(x) (int16_t)((uint16_t)ca[x - 0x88 + 1] << 8 | ca[x - 0x88])
#define A_U8(x) ca[x - 0x88]

#define B_U16_LE(x) (uint16_t)(((uint16_t)cb[x - 0xE1 + 1] << 8 | cb[x - 0xE1]))
#define B_S16_LE(x) (int16_t)(((uint16_t)cb[x - 0xE1 + 1] << 8 | cb[x - 0xE1]))
#define B_S12_HL(x) (int16_t)((uint16_t)cb[x - 0xE1] << 4 | (cb[x - 0xE1 + 1] & 0x0F))
#define B_S12_LH(x) (int16_t)(cb[x - 0xE1] >> 4 | (uint16_t)cb[x - 0xE1 + 1] << 8)
#define B_U8(x) cb[x - 0xE1]

    BME_LOG_PRINT("\r\n");
    _dig_T1 = A_U16_LE(BME_REG_DIG_T1);
    _dig_T2 = A_S16_LE(BME_REG_DIG_T2);
    _dig_T3 = A_S16_LE(BME_REG_DIG_T3);
    BME_LOG_PRINTF("T %04X %04X %04X\r\n", _dig_T1, _dig_T2, _dig_T3);

    _dig_P1 = A_U16_LE(BME_REG_DIG_P1);
    _dig_P2 = A_S16_LE(BME_REG_DIG_P2);
    _dig_P3 = A_S16_LE(BME_REG_DIG_P3);
    _dig_P4 = A_S16_LE(BME_REG_DIG_P4);
    _dig_P5 = A_S16_LE(BME_REG_DIG_P5);
    _dig_P6 = A_S16_LE(BME_REG_DIG_P6);
    _dig_P7 = A_S16_LE(BME_REG_DIG_P7);
    _dig_P8 = A_S16_LE(BME_REG_DIG_P8);
    _dig_P9 = A_S16_LE(BME_REG_DIG_P9);
    BME_LOG_PRINTF("P %04X %04X %04X\r\n", _dig_P1, _dig_P2, _dig_P3);

    _dig_H1 = A_U8(BME_REG_DIG_H1);
    _dig_H2 = B_S16_LE(BME_REG_DIG_H2);
    _dig_H3 = B_U8(BME_REG_DIG_H3);
    _dig_H4 = B_S12_HL(BME_REG_DIG_H4);
    _dig_H5 = B_S12_LH(BME_REG_DIG_H5);
    _dig_H6 = (int8_t)B_U8(BME_REG_DIG_H6);
    BME_LOG_PRINTF("H %02hX %04X %02hX %04X %04X %02hX\r\n", _dig_H1, _dig_H2, _dig_H3, _dig_H4, _dig_H5, _dig_H6);

    return true;
}

bool LeanBME280::readMeasurements()
{
    const unsigned NBYTES = 8;
    uint8_t regs[NBYTES];

    BME_LOG_PRINT(" rM:");
    if (8 != Wire.requestFrom(_i2cAddr, NBYTES, 0xF7, 1, true))
    {
        _wireError = 4;
        BME_LOG_PRINT(" errR ");
        return false;
    }
    BME_LOG_PRINT("ok ");
    for (uint8_t i = 0; i < NBYTES; i++)
        regs[i] = Wire.read();

    _rawP = ((uint32_t)regs[0] << 12L) | ((uint32_t)regs[1] << 4L) | ((uint32_t)regs[2] >> 4);
    _rawT = ((uint32_t)regs[3] << 12L) | ((uint32_t)regs[4] << 4L) | ((uint32_t)regs[5] >> 4);
    _rawH = ((uint32_t)regs[6] << 8L) | ((uint32_t)regs[7]);

    return true;
}

/**
 * @brief Set sampling registers
 * @param mode          Sampling mode (sleep or forced or normal)
 * @param tempSampling  Temperature sampling factor
 * @param pressSampling Pressure sampling factor
 * @param humSampling   Humidity sampling factor
 * @param filter        Filter type
 * @param standbyDuration  delay to standby after measurement
 */
void LeanBME280::setSampling(
    BmeMode mode,
    BmeSampling tempSampling,
    BmeSampling pressSampling,
    BmeSampling humSampling,
    BmeFilter filter,
    BmeStandby standbyDuration)
{
    // Set in sleep mode to provide write access to the “config” register
    write8(BME_REG_CTRL_MEAS, BME280_MODE_SLEEP);

    if (_chipID == CHIP_ID_BME280)
    {
        // See datasheet 5.4.3 Register 0xF2 “ctrl_hum”
        write8(BME_REG_CTRL_HUM, humSampling);
    }
    // See datasheet 5.4.6 Register 0xF5 “config”
    write8(BME_REG_CONFIG, (standbyDuration << 5) | (filter << 2));
    // See datasheet 5.4.5 Register 0xF4 “ctrl_meas”
    _ctrlmeas = (tempSampling << 5) | (pressSampling << 2);
    write8(BME_REG_CTRL_MEAS, _ctrlmeas | mode);
}

#pragma endregion
//=============================================================================
#pragma region Low - level register access

/**
 * @brief Read from 8-bit register.
 * Sets _wireError error code, 0 if transaction was completed without errors
 * @param reg   Register address
 * @return      8-bit register value
 * 
 */
uint8_t LeanBME280::read8(uint8_t reg)
{
    BME_LOG_PRINTF("rb:%02hX=", reg);

    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    if ((_wireError = Wire.endTransmission()))
    {
        BME_LOG_PRINTF("errW(%hd) ", _wireError);
        return 0;
    }
    if (1 != Wire.requestFrom(_i2cAddr, (uint8_t)1))
    {
        _wireError = 4;
        BME_LOG_PRINT("errR ");
        return 0;
    }
    uint8_t value = Wire.read();
    BME_LOG_PRINTF("%02hX ", value);
    return value;
}

/**
 * @brief Write to 8-bit register.
 * Sets _wireError error code, 0 if transaction was completed without errors.
 * @param reg   Register address
 * @param value 8-bit register value.
 *
 */
void LeanBME280::write8(uint8_t reg, uint8_t value)
{
    BME_LOG_PRINTF(" wb:%02hX=%02hX ", reg, value);
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    Wire.write(value);
    _wireError = Wire.endTransmission();
}

#pragma endregion
