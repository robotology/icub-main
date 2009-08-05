#ifndef __temp_sens_interface_h__
#define __temp_sens_interface_h__

#include "dsp56f807.h"

/**
 * The sensor is ready to perform a measurament
 */
#define TEMPERATURE_STATUS_IDLE               0x00
/**
 * The sensor is performing a measurament
 */
#define TEMPERATURE_STATUS_RUNNING            0x01
/**
 * Measurament is finished. Use GetTempSens to get the data
 */
#define TEMPERATURE_STATUS_OK                 0xF0
/**
 * One error occured during the communication with the sensor
 */
#define TEMPERATURE_STATUS_ERR                0xFF

/**
 * One error occured during the communication with the sensor 1/2
 */
#define TEMPERATURE_STATUS_ERR1                0xFE
#define TEMPERATURE_STATUS_ERR2                0xFD


/**
 * Inits the temperature sensor driver
 */
void  init_temp_sens  (void);

/**
 * Performs/continue a temperature measurment
 * @return the current status of the temperature sensor
 * (one ot the TEMPERATURE_STATUS macros)
 */
byte  MeasureTempSens (void);

/**
 * Get the last complete temperature measurament
 * @param sens_num the sensor id to be read (0-1)
 * @return the temperature (°C) measured by the sensor 
 */
Int16 GetTempSens     (byte sens_num);

#endif 