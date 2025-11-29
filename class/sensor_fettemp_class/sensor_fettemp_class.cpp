/*
 * sensor_fettemp_class.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#include <sensor_fettemp_class/sensor_fettemp_class.h>
#include <stdio.h>

// Uncomment for debug output
// #define DEBUG_TEMP

sensor_fettemp_class::sensor_fettemp_class(ADC_HandleTypeDef* adc)
    : hadc(adc), current_temperature(0.0f), current_adc_raw(0), last_update_time(0)
{
    // Initialize ADC if not already done
    if (hadc != nullptr) {
        // ADC already initialized in main.c
        #ifdef DEBUG_TEMP
        printf("FET Temperature Sensor: Initialized\r\n");
        #endif
    }
}

sensor_fettemp_class::~sensor_fettemp_class()
{
    // Cleanup if needed
}

/**
 * @brief Read ADC value from ADC_FETTEMP channel (ADC1_IN3)
 * @return Raw ADC value (0-4095)
 */
uint32_t sensor_fettemp_class::read_adc_value()
{
    uint32_t adc_value = 0;
    
    if (hadc == nullptr) {
        return 0;
    }
    
    // Start ADC conversion
    HAL_ADC_Start(hadc);
    
    // Poll for conversion completion (timeout 100ms)
    if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK) {
        // Read ADC value
        adc_value = HAL_ADC_GetValue(hadc);
    }
    
    // Stop ADC
    HAL_ADC_Stop(hadc);
    
    return adc_value;
}

/**
 * @brief Calculate temperature from ADC value using NTC thermistor formula
 * @param adc_value Raw ADC value (0-4095)
 * @return Temperature in Celsius
 * 
 * Formula:
 * 1. Calculate voltage: V = (ADC / 4095) * 3.3V
 * 2. Calculate NTC resistance: R_ntc = R_series * (V / (Vref - V))
 * 3. Calculate temperature using Beta equation:
 *    1/T = 1/T0 + (1/Beta) * ln(R_ntc / R0)
 */
float sensor_fettemp_class::calculate_temperature(uint32_t adc_value)
{
    if (adc_value == 0 || adc_value >= ADC_RESOLUTION) {
        return -999.0f; // Invalid reading
    }
    
    // Step 1: Calculate voltage
    float voltage = ((float)adc_value / ADC_RESOLUTION) * VREF;
    
    // Step 2: Calculate NTC resistance
    // Voltage divider: V = Vref * (R_ntc / (R_series + R_ntc))
    // Solving for R_ntc: R_ntc = R_series * V / (Vref - V)
    float r_ntc = SERIES_RESISTOR * voltage / (VREF - voltage);
    
    // Step 3: Calculate temperature using Beta equation
    // 1/T = 1/T0 + (1/Beta) * ln(R_ntc / R0)
    float temp_kelvin = 1.0f / ((1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_RESISTANCE));
    
    // Convert to Celsius
    float temp_celsius = temp_kelvin - 273.15f;
    
    #ifdef DEBUG_TEMP
    printf("ADC=%lu, V=%.3fV, R_ntc=%.1fΩ, Temp=%.2f°C\r\n", 
           adc_value, voltage, r_ntc, temp_celsius);
    #endif
    
    return temp_celsius;
}

/**
 * @brief Update temperature reading from ADC
 */
void sensor_fettemp_class::update_temperature()
{
    // Read ADC value
    current_adc_raw = read_adc_value();
    
    // Calculate temperature
    current_temperature = calculate_temperature(current_adc_raw);
    
    // Update timestamp
    last_update_time = HAL_GetTick();
}

/**
 * @brief Get current temperature in Celsius
 * @return Temperature in Celsius
 */
float sensor_fettemp_class::get_temperature()
{
    return current_temperature;
}

/**
 * @brief Get raw ADC value
 * @return Raw ADC value (0-4095)
 */
uint32_t sensor_fettemp_class::get_adc_raw()
{
    return current_adc_raw;
}

/**
 * @brief Get complete temperature data structure
 * @return temp_data_t structure with temperature, ADC raw, and timestamp
 */
temp_data_t sensor_fettemp_class::get_temp_data()
{
    temp_data_t data;
    data.temperature_celsius = current_temperature;
    data.adc_raw = current_adc_raw;
    data.timestamp_ms = last_update_time;
    return data;
}

/**
 * @brief Publish temperature data (for debugging or ROS topic)
 */
void sensor_fettemp_class::publish_temperature()
{
    #ifdef DEBUG_TEMP
    printf("[FET Temp] %.2f°C (ADC: %lu) @ %lu ms\r\n",
           current_temperature, current_adc_raw, last_update_time);
    #endif
    
    // TODO: Publish to ROS2 topic when micro-ros integration is ready
    // Example: std_msgs/msg/Float32 for temperature
}