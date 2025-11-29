/*
 * sensor_fettemp_class.h
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#ifndef SENSOR_FETTEMP_CLASS_SENSOR_FETTEMP_CLASS_H_
#define SENSOR_FETTEMP_CLASS_SENSOR_FETTEMP_CLASS_H_

extern "C" {
#include "main.h"
}

#include <cmath>

// NTC Thermistor parameters
#define NTC_RESISTANCE      10000.0f   // 10K NTC at 25°C
#define NTC_BETA            1360.0f    // Beta value
#define NTC_T0              298.15f    // Reference temperature (25°C in Kelvin)
#define SERIES_RESISTOR     10000.0f   // Series resistor value (10K)
#ifndef ADC_RESOLUTION
#define ADC_RESOLUTION      4095.0f    // 12-bit ADC
#endif
#define VREF                3.3f        // Reference voltage
#define ADC_FETTEMP_CHANNEL ADC_CHANNEL_3  // PA3 -> ADC1_IN3
#define ADC_FETTEMP_RANK    4          // Rank 4 in ADC sequence

typedef struct {
    float temperature_celsius;
    uint32_t adc_raw;
    uint32_t timestamp_ms;
} temp_data_t;

class sensor_fettemp_class
{
private:
    ADC_HandleTypeDef* hadc;
    float current_temperature;
    uint32_t current_adc_raw;
    uint32_t last_update_time;
    
    // Private methods
    uint32_t read_adc_value();
    float calculate_temperature(uint32_t adc_value);
    
public:
    sensor_fettemp_class(ADC_HandleTypeDef* adc);
    virtual ~sensor_fettemp_class();
    
    // Public methods
    void update_temperature();
    float get_temperature();
    uint32_t get_adc_raw();
    temp_data_t get_temp_data();
    void publish_temperature();
};

#endif /* SENSOR_FETTEMP_CLASS_SENSOR_FETTEMP_CLASS_H_ */
