/*
 * task_class.h
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#ifndef TASK_CLASS_TASK_CLASS_H_
#define TASK_CLASS_TASK_CLASS_H_

extern "C" {
#include "cmsis_os.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
}

#include "../microros_class/microros_class.h"
#include "../pwm_dcmotor_class/pwm_dcmotor_class.h"
#include "../sensor_encoder_class/sensor_encoder_class.h"
#include "../DataClass/DataClass.h"
#include "../sensor_imu_class/sensor_imu_class.h"
#include "../sensor_fettemp_class/sensor_fettemp_class.h"
#include "../led_class/led_class.h"

// Data structures for Queue communication
typedef struct {
  float linear_x;
  float linear_y;
  float angular_z;
} CmdVelData_t;

typedef struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
} ImuData_t;

typedef struct {
  int32_t left_encoder;
  int32_t right_encoder;
} EncoderData_t;

typedef struct {
  float temperature;
  uint32_t adc_raw;
} FetTempData_t;

class task_class
{
private:
    // Global object pointers
    microros_class* pMicroRos;
    pwm_dcmotor_class* pPwmMotor;
    sensor_encoder_class* pEncoder;
    DataClass* pDataClass;
    sensor_imu_class* pImuSensor;
    sensor_fettemp_class* pFetTemp;
    led_class* pLed;
    
    // Queue handles
    osMessageQueueId_t cmdVelQueueHandle;
    osMessageQueueId_t imuDataQueueHandle;
    osMessageQueueId_t encoderDataQueueHandle;
    osMessageQueueId_t fetTempQueueHandle;
    
    // UART and SPI handles
    UART_HandleTypeDef* huart;
    SPI_HandleTypeDef* hspi;

public:
    task_class(UART_HandleTypeDef* uart, SPI_HandleTypeDef* spi);
    virtual ~task_class();
    
    // Initialize queues
    bool initQueues();
    
    // Task functions
    void runMicroRosTask();
    void runControlTask();
    void runSensorTask();
    
    // Static wrappers for FreeRTOS
    static void microRosTaskWrapper(void* argument);
    static void controlTaskWrapper(void* argument);
    static void sensorTaskWrapper(void* argument);
    
    // Static instance pointer for callbacks
    static task_class* instance;
};

#endif /* TASK_CLASS_TASK_CLASS_H_ */
