/**
 * @file LeanBME280.ch
 * @author Bernd Waldmann
 * @date 2022-02-26
 *
 * This Revision: $Id: $
 */

#ifndef LEAN_BME280_H_
#define LEAN_BME280_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address
#define BME280_I2C_ADDR 0x76     ///< I2C address
#define BME280_I2C_ADDR_ALT 0x77 ///< I2C alternative address

/** Sleep mode register values, sleep/forced/normal */
typedef enum
{
    BME280_MODE_SLEEP = 0,  ///< Sleep mode
    BME280_MODE_FORCED = 1, ///< Forced mode
    BME280_MODE_NORMAL = 3  ///< Normal mode
} BmeMode;

/** Sampling registers ctrl_hum, ctrl_meas values, x1/2/4/8/16 or off */
typedef enum
{
    BME280_SAMPLING_NONE = 0, ///< Sampling disabled
    BME280_SAMPLING_X1 = 1,   ///< x1 Sampling
    BME280_SAMPLING_X2 = 2,   ///< x2 Sampling
    BME280_SAMPLING_X4 = 3,   ///< x4 Sampling
    BME280_SAMPLING_X8 = 4,   ///< x8 Sampling
    BME280_SAMPLING_X16 = 5   ///< x16 Sampling
} BmeSampling;

/** Filter register values: x1/2/4/8/16 */
typedef enum
{
    BME280_FILTER_OFF = 0, ///< Filter off
    BME280_FILTER_X2 = 1,  ///< x2 Filter
    BME280_FILTER_X4 = 2,  ///< x4 Filter
    BME280_FILTER_X8 = 3,  ///< x8 Filter
    BME280_FILTER_X16 = 4  ///< x16 Filter
} BmeFilter;

/** Standby config register values, 0.5ms..1s */
typedef enum
{
    BME280_STANDBY_MS_0_5 = 0,  ///< 0.5m standby
    BME280_STANDBY_MS_10 = 6,   ///< 10ms standby
    BME280_STANDBY_MS_20 = 7,   ///< 20ms standby
    BME280_STANDBY_MS_62_5 = 1, ///< 62.5 standby
    BME280_STANDBY_MS_125 = 2,  ///< 125ms standby
    BME280_STANDBY_MS_250 = 3,  ///< 250ms standby
    BME280_STANDBY_MS_500 = 4,  ///< 500ms standby
    BME280_STANDBY_MS_1000 = 5  ///< 1s standby
} BmeStandby;

/**
 * @brief BME280 sensor class
 */
class LeanBME280
{
  public:
    static const int INAN = -32000; ///< value returned as error indicator

    LeanBME280(uint8_t i2cAddr = BME280_I2C_ADDR);

    bool begin();

    void takeForcedMeasurementNoWait();
    void takeForcedMeasurement();

    bool readSensor(int16_t *pTemp, int16_t *pHum = NULL, int16_t *pBaro = NULL);

    int16_t readTemperature();  
    int16_t readHumidity();     
    int16_t _readHumidity();    
    int16_t readPressure();     
    int16_t _readPressure();    
    uint8_t wireError() { return _wireError; } ///< 0 if ok

    void setSampling(
        BmeMode mode = BME280_MODE_FORCED,
        BmeSampling tempSampling = BME280_SAMPLING_X16,
        BmeSampling pressSampling = BME280_SAMPLING_X16,
        BmeSampling humSampling = BME280_SAMPLING_X16,
        BmeFilter filter = BME280_FILTER_OFF,
        BmeStandby standbyDuration = BME280_STANDBY_MS_0_5);

  protected:
    // low-level register access
    uint8_t read8(uint8_t reg);
    void write8(uint8_t reg, uint8_t value);

    uint8_t _i2cAddr; ///< I2C address
    uint8_t _chipID;  ///< Chip iD
    uint8_t _wireError;
    int32_t _t_fine; ///< Temperature variable
    uint8_t _ctrlmeas;

    int32_t _rawT;
    int32_t _rawH;
    int32_t _rawP;

    // calibration coefficients
    uint16_t _dig_T1;
    int16_t _dig_T2;
    int16_t _dig_T3;

    uint16_t _dig_P1;
    int16_t _dig_P2;
    int16_t _dig_P3;
    int16_t _dig_P4;
    int16_t _dig_P5;
    int16_t _dig_P6;
    int16_t _dig_P7;
    int16_t _dig_P8;
    int16_t _dig_P9;

    uint8_t _dig_H1;
    int16_t _dig_H2;
    uint8_t _dig_H3;
    int16_t _dig_H4;
    int16_t _dig_H5;
    int8_t _dig_H6;

    bool readCoefficients();
    bool readMeasurements();
};

#endif // LEAN_BME280_H_
