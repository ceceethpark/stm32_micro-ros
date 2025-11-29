/*
 * sensor_imu_class.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#include "sensor_imu_class.h"
#include <math.h>

sensor_imu_class::sensor_imu_class(SPI_HandleTypeDef* spi, GPIO_TypeDef* cs_gpio, uint16_t cs_pin)
    : hspi(spi), cs_port(cs_gpio), cs_pin(cs_pin)
{
    // ICM-42670 scale factors
    // Accel: ±16g range, 2048 LSB/g
    accel_scale = 9.81f / 2048.0f;  // Convert to m/s^2
    
    // Gyro: ±2000 dps range, 16.4 LSB/(dps)
    gyro_scale = (M_PI / 180.0f) / 16.4f;  // Convert to rad/s
}

sensor_imu_class::~sensor_imu_class()
{
}

void sensor_imu_class::csLow()
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

void sensor_imu_class::csHigh()
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

uint8_t sensor_imu_class::readRegister(uint8_t reg)
{
    uint8_t tx_data = reg | 0x80;  // Read: set MSB to 1
    uint8_t rx_data = 0;
    
    csLow();
    HAL_SPI_Transmit(hspi, &tx_data, 1, 100);
    HAL_SPI_Receive(hspi, &rx_data, 1, 100);
    csHigh();
    
    return rx_data;
}

void sensor_imu_class::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F;  // Write: clear MSB
    tx_data[1] = value;
    
    csLow();
    HAL_SPI_Transmit(hspi, tx_data, 2, 100);
    csHigh();
}

void sensor_imu_class::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length)
{
    uint8_t tx_data = reg | 0x80;  // Read: set MSB to 1
    
    csLow();
    HAL_SPI_Transmit(hspi, &tx_data, 1, 100);
    HAL_SPI_Receive(hspi, buffer, length, 100);
    csHigh();
}

bool sensor_imu_class::init()
{
    // CS pin high (inactive)
    csHigh();
    HAL_Delay(10);
    
    // Check WHO_AM_I
    uint8_t whoami = getWhoAmI();
    if (whoami != ICM42670_WHO_AM_I_VALUE) {
        return false;  // Wrong device ID
    }
    
    // Soft reset
    writeRegister(ICM42670_PWR_MGMT0, 0x00);
    HAL_Delay(50);
    
    // Power on Accel and Gyro in Low Noise mode
    // Accel: Low Noise, Gyro: Low Noise
    writeRegister(ICM42670_PWR_MGMT0, 0x0F);
    HAL_Delay(50);
    
    // Configure Accel: ±16g, ODR 1kHz
    writeRegister(ICM42670_ACCEL_CONFIG0, 0x06);  // ±16g range, ODR 1kHz
    
    // Configure Gyro: ±2000dps, ODR 1kHz
    writeRegister(ICM42670_GYRO_CONFIG0, 0x06);   // ±2000dps range, ODR 1kHz
    
    HAL_Delay(50);
    
    return true;
}

bool sensor_imu_class::readImuData(ImuRawData_t* data)
{
    if (data == nullptr) {
        return false;
    }
    
    uint8_t buffer[14];  // Temp(2) + Accel(6) + Gyro(6)
    
    // Read all sensor data starting from TEMP_DATA1
    readRegisters(ICM42670_TEMP_DATA1, buffer, 14);
    
    // Parse temperature (optional)
    int16_t temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->temperature = (float)temp_raw / 128.0f + 25.0f;  // ICM-42670 temp formula
    
    // Parse accelerometer data (16-bit signed)
    int16_t accel_x_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accel_y_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t accel_z_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    // Parse gyroscope data (16-bit signed)
    int16_t gyro_x_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gyro_y_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gyro_z_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // Apply scale factors
    data->accel_x = (float)accel_x_raw * accel_scale;
    data->accel_y = (float)accel_y_raw * accel_scale;
    data->accel_z = (float)accel_z_raw * accel_scale;
    
    data->gyro_x = (float)gyro_x_raw * gyro_scale;
    data->gyro_y = (float)gyro_y_raw * gyro_scale;
    data->gyro_z = (float)gyro_z_raw * gyro_scale;
    
    return true;
}

uint8_t sensor_imu_class::getWhoAmI()
{
    return readRegister(ICM42670_WHO_AM_I);
}
