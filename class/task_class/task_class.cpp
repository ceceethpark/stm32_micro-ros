/*
 * task_class.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#include "task_class.h"
#include "extern.h"

// Static instance pointer
task_class* task_class::instance = nullptr;

task_class::task_class(UART_HandleTypeDef* uart, SPI_HandleTypeDef* spi)
    : pMicroRos(nullptr), pPwmMotor(nullptr), pEncoder(nullptr), pDataClass(nullptr), pImuSensor(nullptr),
      pFetTemp(nullptr), pLed(nullptr), cmdVelQueueHandle(nullptr), imuDataQueueHandle(nullptr), 
      encoderDataQueueHandle(nullptr), fetTempQueueHandle(nullptr), huart(uart), hspi(spi)
{
    // Set static instance
    instance = this;
    
    printf("[INIT] TaskManager constructor - minimal init\r\n");
    
    // Initialize IMU Sensor (SPI1, CS pin: PA4 from pinout)
    pImuSensor = new sensor_imu_class(hspi, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    printf("[INIT] IMU sensor created at: %p\r\n", pImuSensor);
}

task_class::~task_class()
{
    if (pMicroRos != nullptr) delete pMicroRos;
    if (pPwmMotor != nullptr) delete pPwmMotor;
    if (pEncoder != nullptr) delete pEncoder;
    if (pDataClass != nullptr) delete pDataClass;
    if (pImuSensor != nullptr) delete pImuSensor;
    if (pFetTemp != nullptr) delete pFetTemp;
    if (pLed != nullptr) delete pLed;
}

bool task_class::initQueues()
{
    // Create message queues for inter-task communication
    cmdVelQueueHandle = osMessageQueueNew(5, sizeof(CmdVelData_t), NULL);
    imuDataQueueHandle = osMessageQueueNew(5, sizeof(ImuData_t), NULL);
    encoderDataQueueHandle = osMessageQueueNew(5, sizeof(EncoderData_t), NULL);
    fetTempQueueHandle = osMessageQueueNew(5, sizeof(FetTempData_t), NULL);
    
    return (cmdVelQueueHandle != nullptr && 
            imuDataQueueHandle != nullptr && 
            encoderDataQueueHandle != nullptr &&
            fetTempQueueHandle != nullptr);
}

void task_class::runMicroRosTask()
{
    printf("[MICROROS] Start\r\n");
    
    uint32_t microros_count = 0;
    
    // MicroROS agent 연결 (UART2, 921600 baud)
    printf("[MICROROS] Waiting for agent...\r\n");
    pMicroRos = new microros_class(huart, "stm32_node", "int32_publisher", "int32_subscriber", "cmd_vel");
    
    if (!pMicroRos->init()) {
        printf("[MICROROS] Init failed!\r\n");
        Error_Handler();
    }
    
    printf("[MICROROS] Agent connected!\r\n");
    
    for(;;)
    {
        // Executor spin
        if (pMicroRos != nullptr) {
            pMicroRos->spin();
        }
        
        // Queue 비우기
        ImuData_t imuData;
        osMessageQueueGet(imuDataQueueHandle, &imuData, NULL, 0);
        
        EncoderData_t encoderData;
        osMessageQueueGet(encoderDataQueueHandle, &encoderData, NULL, 0);
        
        FetTempData_t fetTempData;
        osMessageQueueGet(fetTempQueueHandle, &fetTempData, NULL, 0);
        
        // Debug counter publish
        if (pDataClass != nullptr && pMicroRos != nullptr) {
            int32_t data = pDataClass->getData();
            pMicroRos->publish(data);
            pDataClass->incrementData();
            
            if (microros_count % 100 == 0) {
                printf("[MICROROS] Publishing: %ld\r\n", data);
            }
        }
        
        microros_count++;
        osDelay(50);
    }
}

void task_class::runControlTask()
{
    printf("[CONTROL] Start\r\n");
    
    osDelay(100);
    printf("[CONTROL] Init done\r\n");
    
    pPwmMotor = nullptr;
    pEncoder = nullptr;
    uint32_t control_count = 0;
    
    for(;;)
    {
        if (control_count % 50 == 0) {
            printf("[CONTROL] Count: %lu\r\n", control_count);
        }
        
        control_count++;
        osDelay(10);
    }
}

void task_class::runSensorTask()
{
    printf("[SENSOR] Start\r\n");
    
    uint32_t sensor_count = 0;
    
    printf("[SENSOR] Loop\r\n");
    
    for(;;)
    {
        if (sensor_count % 10 == 0) {
            printf("[SENSOR] Count: %lu\r\n", sensor_count);
        }
        
        sensor_count++;
        osDelay(50);
    }
}

// Static wrapper functions for FreeRTOS
void task_class::microRosTaskWrapper(void* argument)
{
    if (instance != nullptr) {
        instance->runMicroRosTask();
    }
}

void task_class::controlTaskWrapper(void* argument)
{
    if (instance != nullptr) {
        instance->runControlTask();
    }
}

void task_class::sensorTaskWrapper(void* argument)
{
    if (instance != nullptr) {
        instance->runSensorTask();
    }
}
