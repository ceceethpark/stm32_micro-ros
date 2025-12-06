/*
 * task_class.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#include "task_class.h"
#include "extern.h"
#include <math.h>  // For sinf, cosf

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
    
    // Initialize Data Class for counter publishing
    pDataClass = new DataClass();
    printf("[INIT] DataClass created at: %p\r\n", pDataClass);
    
    // Initialize LED controller
    pLed = new led_class();
    printf("[INIT] LED controller created at: %p\r\n", pLed);
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
    osDelay(200);  // Wait for other tasks to print
    printf("[BRIDGE] UART-JSON Bridge Task Started (DMA Mode)\r\n");
    printf("[BRIDGE] Bidirectional: IMU->ROS2, cmd_vel<-ROS2\r\n");
    
    // DMA circular buffer for UART RX
    static uint8_t dma_rx_buffer[512];
    static char json_rx_buffer[512];
    static uint16_t json_rx_pos = 0;
    static uint32_t old_dma_pos = 0;
    
    // Start DMA reception in circular mode
    HAL_UART_Receive_DMA(huart, dma_rx_buffer, sizeof(dma_rx_buffer));
    
    // Enable UART IDLE interrupt for frame detection
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    
    printf("[BRIDGE] DMA started, buffer size: %d\r\n", sizeof(dma_rx_buffer));
    
    // Initialize DMA position to current counter (skip any junk data)
    old_dma_pos = sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    printf("[BRIDGE] Initial DMA pos: %lu\r\n", old_dma_pos);
    
    uint32_t count = 0;
    float dummy_x = 0.0f;
    float dummy_y = 0.0f;
    float dummy_z = 9.81f;  // Gravity
    
    for(;;)
    {
        // Check DMA buffer for new data
        uint32_t current_dma_pos = sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        
        // Process new bytes from DMA buffer
        while (old_dma_pos != current_dma_pos) {
            uint8_t byte = dma_rx_buffer[old_dma_pos];
            old_dma_pos = (old_dma_pos + 1) % sizeof(dma_rx_buffer);
            
            // Skip null bytes (no longer needed with Byte mode DMA)
            if (byte == 0x00) {
                continue;
            }
            
            // Accumulate bytes until closing brace
            if (byte == '}' && json_rx_pos > 0 && json_rx_buffer[0] == '{') {
                // Add closing brace and process
                json_rx_buffer[json_rx_pos++] = byte;
                json_rx_buffer[json_rx_pos] = '\0';
                
                // Parse JSON cmd_vel message
                // Support both formats:
                // 1. {"cmd_vel":{"linear":0.5,"angular":0.2}}
                // 2. {"linear": 1.0, "angular": 0.5}
                float linear = 0.0f, angular = 0.0f;
                
                // Simple JSON parsing (find "linear" and "angular" keys)
                char* linear_str = strstr(json_rx_buffer, "\"linear\"");
                char* angular_str = strstr(json_rx_buffer, "\"angular\"");
                
                if (linear_str != nullptr) {
                    // Skip to the colon and parse the number
                    char* colon = strchr(linear_str, ':');
                    if (colon != nullptr) {
                        sscanf(colon + 1, "%f", &linear);
                    }
                }
                if (angular_str != nullptr) {
                    // Skip to the colon and parse the number
                    char* colon = strchr(angular_str, ':');
                    if (colon != nullptr) {
                        sscanf(colon + 1, "%f", &angular);
                    }
                }
                
                // Send to control task queue
                CmdVelData_t cmdVel;
                cmdVel.linear_x = linear;
                cmdVel.angular_z = angular;
                osMessageQueuePut(cmdVelQueueHandle, &cmdVel, 0, 0);
                
                // Print every 50th cmd_vel (reduce console noise)
                static uint32_t rx_count = 0;
                if (++rx_count % 50 == 0) {
                    printf("[BRIDGE] RX #%lu cmd_vel: L=%.2f, A=%.2f\r\n", rx_count, linear, angular);
                }
                
                json_rx_pos = 0;  // Reset buffer
            } else if (byte == '{') {
                // Start new JSON message
                json_rx_pos = 0;
                json_rx_buffer[json_rx_pos++] = byte;
            } else if (json_rx_pos > 0 && json_rx_pos < sizeof(json_rx_buffer) - 1) {
                // Continue accumulating
                json_rx_buffer[json_rx_pos++] = byte;
            }
        }
        
        // Read IMU data from queue (if available)
        ImuData_t imuData;
        if (osMessageQueueGet(imuDataQueueHandle, &imuData, NULL, 0) == osOK) {
            // Use real data if available
            dummy_x = imuData.accel_x;
            dummy_y = imuData.accel_y;
            dummy_z = imuData.accel_z;
        } else {
            // Generate dummy data (sine wave for testing)
            dummy_x = sinf((float)count * 0.1f) * 2.0f;
            dummy_y = cosf((float)count * 0.1f) * 2.0f;
            dummy_z = 9.81f + sinf((float)count * 0.05f) * 0.5f;
        }
        
        // Send JSON via UART2
        char json_buf[128];
        int len = snprintf(json_buf, sizeof(json_buf),
                          "{\"imu\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"cnt\":%lu}\n",
                          dummy_x, dummy_y, dummy_z, count);
        
        if (len > 0 && len < sizeof(json_buf)) {
            HAL_UART_Transmit(huart, (uint8_t*)json_buf, len, 100);
        }
        
        // Clear other queues
        EncoderData_t encoderData;
        osMessageQueueGet(encoderDataQueueHandle, &encoderData, NULL, 0);
        
        FetTempData_t fetTempData;
        osMessageQueueGet(fetTempQueueHandle, &fetTempData, NULL, 0);
        
        // Heartbeat every 2 seconds
        if (count % 10 == 0) {
            printf("[BRIDGE] %lu sent\r\n", count);
        }
        
        count++;
        osDelay(200);  // Send at 5Hz
    }
}

void task_class::runControlTask()
{
    osDelay(50);  // Wait briefly
    printf("[CONTROL] Start\r\n");
    osDelay(10);
    
    // Initialize motor controller (ADC1 for current sensing, TIM1 for PWM)
    pPwmMotor = new pwm_dcmotor_class(&hadc1, &htim1);
    printf("[CONTROL] Motor OK\r\n");
    osDelay(10);
    
    // Initialize encoders (TIM3, TIM4)
    pEncoder = new sensor_encoder_class(&htim3, &htim4);
    printf("[CONTROL] Encoder OK\r\n");
    osDelay(10);
    
    printf("[CONTROL] Ready\r\n");
    
    uint32_t control_count = 0;
    CmdVelData_t cmdVel = {0};
    
    printf("[CONTROL] Entering loop\r\n");
    
    for(;;)
    {
        // Check for new cmd_vel commands (non-blocking)
        osStatus_t status = osMessageQueueGet(cmdVelQueueHandle, &cmdVel, NULL, 0);
        
        if (status == osOK) {
            // Convert cmd_vel to motor commands
            // Differential drive kinematics:
            // left_speed = linear_x - angular_z * wheel_base / 2
            // right_speed = linear_x + angular_z * wheel_base / 2
            
            float left_speed = cmdVel.linear_x - cmdVel.angular_z * 0.15f;  // 0.15m wheel base
            float right_speed = cmdVel.linear_x + cmdVel.angular_z * 0.15f;
            
            // Set motor speeds (convert to -100 to 100 range)
            if (pPwmMotor != nullptr) {
                pPwmMotor->PwmMotor_Set_Speed(MOTOR_1, (int)(left_speed * 100));
                pPwmMotor->PwmMotor_Set_Speed(MOTOR_2, (int)(right_speed * 100));
            }
            
            // Print every 50th command (reduce noise)
            static uint32_t cmd_count = 0;
            if (++cmd_count % 50 == 0) {
                printf("[CONTROL] cmd_vel: L=%.2f, R=%.2f\r\n", left_speed, right_speed);
            }
        }
        
        // Read encoder values
        if (pEncoder != nullptr) {
            EncoderData_t encoderData;
            encoderData.left_encoder = pEncoder->get_raw_count(ENCODER_MOTOR_1);
            encoderData.right_encoder = pEncoder->get_raw_count(ENCODER_MOTOR_2);
            
            // Send to MicroROS task for publishing
            osMessageQueuePut(encoderDataQueueHandle, &encoderData, 0, 0);
        }
        
        control_count++;
        
        // Heartbeat every 500ms for visibility
        if (control_count % 50 == 0) {
            printf("[CTL] %lu\r\n", control_count);
        }
        
        osDelay(10);  // 100Hz control loop
    }
}

void task_class::runSensorTask()
{
    osDelay(100);  // Wait for other tasks
    printf("[SENSOR] Start\r\n");
    osDelay(10);
    
    // Skip IMU initialization (hardware not connected)
    bool imu_ready = false;
    
    // Initialize FET temperature sensor (ADC1)
    pFetTemp = new sensor_fettemp_class(&hadc1);
    printf("[SENSOR] Ready\r\n");
    osDelay(10);
    
    uint32_t sensor_count = 0;
    
    printf("[SENSOR] Entering loop\r\n");
    
    for(;;)
    {
        // Read IMU data (only if initialized successfully)
        if (imu_ready && pImuSensor != nullptr) {
            ImuRawData_t rawData;
            
            if (pImuSensor->readImuData(&rawData)) {
                ImuData_t imuData;
                imuData.accel_x = rawData.accel_x;
                imuData.accel_y = rawData.accel_y;
                imuData.accel_z = rawData.accel_z;
                imuData.gyro_x = rawData.gyro_x;
                imuData.gyro_y = rawData.gyro_y;
                imuData.gyro_z = rawData.gyro_z;
                
                // Send to MicroROS task for publishing
                osMessageQueuePut(imuDataQueueHandle, &imuData, 0, 0);
                
                if (sensor_count % 20 == 0) {
                    printf("[SENSOR] IMU: Ax=%d, Ay=%d, Az=%d\r\n", 
                           (int)imuData.accel_x, (int)imuData.accel_y, (int)imuData.accel_z);
                }
            }
        }
        
        // Heartbeat every 500ms
        if (sensor_count % 10 == 0) {
            printf("[SEN] %lu\r\n", sensor_count);
        }
        
        // Read FET temperature
        if (pFetTemp != nullptr && sensor_count % 10 == 0) {  // Read every 500ms
            pFetTemp->update_temperature();
            FetTempData_t fetTempData;
            fetTempData.temperature = pFetTemp->get_temperature();
            fetTempData.adc_raw = pFetTemp->get_adc_raw();
            
            // Send to MicroROS task
            osMessageQueuePut(fetTempQueueHandle, &fetTempData, 0, 0);
            
            printf("[SENSOR] Temperature: %.1f°C\r\n", fetTempData.temperature);
        }
        
        sensor_count++;
        osDelay(50);  // 20Hz sensor reading
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