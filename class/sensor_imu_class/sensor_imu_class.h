/*
 * sensor_imu_class.h
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#ifndef SENSOR_IMU_CLASS_SENSOR_IMU_CLASS_H_
#define SENSOR_IMU_CLASS_SENSOR_IMU_CLASS_H_

extern "C" {
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
}

// ICM-42670 Register Map
#define ICM42670_WHO_AM_I           0x75
#define ICM42670_WHO_AM_I_VALUE     0x67

#define ICM42670_PWR_MGMT0          0x4E
#define ICM42670_GYRO_CONFIG0       0x4F
#define ICM42670_ACCEL_CONFIG0      0x50
#define ICM42670_GYRO_CONFIG1       0x51
#define ICM42670_ACCEL_CONFIG1      0x53

#define ICM42670_TEMP_DATA1         0x1D
#define ICM42670_TEMP_DATA0         0x1E
#define ICM42670_ACCEL_DATA_X1      0x1F
#define ICM42670_ACCEL_DATA_X0      0x20
#define ICM42670_ACCEL_DATA_Y1      0x21
#define ICM42670_ACCEL_DATA_Y0      0x22
#define ICM42670_ACCEL_DATA_Z1      0x23
#define ICM42670_ACCEL_DATA_Z0      0x24
#define ICM42670_GYRO_DATA_X1       0x25
#define ICM42670_GYRO_DATA_X0       0x26
#define ICM42670_GYRO_DATA_Y1       0x27
#define ICM42670_GYRO_DATA_Y0       0x28
#define ICM42670_GYRO_DATA_Z1       0x29
#define ICM42670_GYRO_DATA_Z0       0x2A

// IMU Data structure
typedef struct {
    float accel_x;  // m/s^2
    float accel_y;
    float accel_z;
    float gyro_x;   // rad/s
    float gyro_y;
    float gyro_z;
    float temperature; // °C
} ImuRawData_t;

class sensor_imu_class
{
private:
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    
    // Scale factors
    float accel_scale;  // LSB to m/s^2
    float gyro_scale;   // LSB to rad/s
    
    // SPI communication functions
    void csLow();
    void csHigh();
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);

public:
    sensor_imu_class(SPI_HandleTypeDef* spi, GPIO_TypeDef* cs_gpio, uint16_t cs_pin);
    virtual ~sensor_imu_class();
    
    // Initialization
    bool init();
    
    // Read IMU data
    bool readImuData(ImuRawData_t* data);
    
    // Get WHO_AM_I register
    uint8_t getWhoAmI();
};

#endif /* SENSOR_IMU_CLASS_SENSOR_IMU_CLASS_H_ */
