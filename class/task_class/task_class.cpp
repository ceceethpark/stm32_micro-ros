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
    
    // Initialize DataClass
    pDataClass = new DataClass(100);
    pDataClass->setTemperature(25.5f);
    pDataClass->setIsActive(true);
    
    // Initialize IMU Sensor (SPI1, CS pin: PB0 from pinout image)
    pImuSensor = new sensor_imu_class(hspi, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    
    // Initialize LED
    pLed = new led_class();
    // Start heartbeat blink on GREEN LED (1Hz)
    pLed->led_blink(LED_GREEN, 1000);
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
    // MicroRos 객체 생성 및 초기화
    pMicroRos = new microros_class(huart, "stm32f405_node", 
                                   "debug_counter", nullptr, "cmd_vel");
    
    if (!pMicroRos->init()) {
        Error_Handler();
    }

    // cmd_vel 콜백 설정 - Queue로 데이터 전달
    pMicroRos->setCmdVelCallback([](const void* msg) {
        const geometry_msgs__msg__Twist* twist = (const geometry_msgs__msg__Twist*)msg;
        
        CmdVelData_t cmdVelData;
        cmdVelData.linear_x = twist->linear.x;
        cmdVelData.linear_y = twist->linear.y;
        cmdVelData.angular_z = twist->angular.z;
        
        // Queue에 cmd_vel 데이터 전송 (ControlTask로)
        if (instance && instance->cmdVelQueueHandle) {
            osMessageQueuePut(instance->cmdVelQueueHandle, &cmdVelData, 0, 0);
        }
    });

    // Main loop
    for(;;)
    {
        // cmd_vel 메시지 처리 (executor spin)
        if (pMicroRos != nullptr) {
            pMicroRos->spin();
        }
        
        // Queue에서 IMU 데이터 받아서 publish
        ImuData_t imuData;
        if (osMessageQueueGet(imuDataQueueHandle, &imuData, NULL, 0) == osOK) {
            // TODO: Publish IMU data to ROS
            // pMicroRos->publishImu(&imuData);
        }
        
        // Queue에서 Encoder 데이터 받아서 publish
        EncoderData_t encoderData;
        if (osMessageQueueGet(encoderDataQueueHandle, &encoderData, NULL, 0) == osOK) {
            // TODO: Publish Encoder data to ROS
            // pMicroRos->publishEncoder(&encoderData);
        }
        
        // Queue에서 FET Temperature 데이터 받아서 publish
        FetTempData_t fetTempData;
        if (osMessageQueueGet(fetTempQueueHandle, &fetTempData, NULL, 0) == osOK) {
            // TODO: Publish FET Temperature to ROS (std_msgs/Float32)
            // pMicroRos->publishFetTemp(fetTempData.temperature);
        }
        
        // Debug counter publish
        if (pDataClass != nullptr && pMicroRos != nullptr) {
            int32_t data = pDataClass->getData();
            pMicroRos->publish(data);
            pDataClass->incrementData();
        }
        
        osDelay(50);  // 20Hz publish rate
    }
}

void task_class::runControlTask()
{
    // PWM Motor 초기화 (ADC1 for current sensing, TIM1 for PWM)
    pPwmMotor = new pwm_dcmotor_class(&hadc1, &htim1);
    
    // Encoder 초기화 (Timer3 for Motor1, Timer4 for Motor2)
    pEncoder = new sensor_encoder_class(&htim3, &htim4, ENCODER_PPR);
    
    // 초기화 대기
    osDelay(100);
    
    CmdVelData_t cmdVelData;
    cmdVelData.linear_x = 0.0f;
    cmdVelData.linear_y = 0.0f;
    cmdVelData.angular_z = 0.0f;
    
    // Main loop - 10ms cycle (100Hz)
    for(;;)
    {
        // Queue에서 cmd_vel 데이터 수신 (non-blocking)
        if (osMessageQueueGet(cmdVelQueueHandle, &cmdVelData, NULL, 0) == osOK) {
            // 새로운 cmd_vel 수신됨
        }
        
        // cmd_vel 데이터를 모터 속도로 변환
        if (pPwmMotor != nullptr) {
            // linear.x: 전진/후진 속도 (-1.0 ~ +1.0)
            // angular.z: 회전 속도 (-1.0 ~ +1.0)
            
            int speed_left = (int)((cmdVelData.linear_x - cmdVelData.angular_z) * 100.0f);
            int speed_right = (int)((cmdVelData.linear_x + cmdVelData.angular_z) * 100.0f);
            
            pPwmMotor->PwmMotor_Set_Speed(MOTOR_1, speed_left);    // Left motor
            pPwmMotor->PwmMotor_Set_Speed(MOTOR_2, speed_right);  // Right motor
            
            // PWM 업데이트
            pPwmMotor->pwm_update();
        }
        
        // Encoder RPM 업데이트 및 퍼블리시
        if (pEncoder != nullptr) {
            pEncoder->update_rpm();
            pEncoder->publish_rpm();
        }
        
        // LED 업데이트 (blink 처리)
        if (pLed != nullptr) {
            pLed->update();
        }
        
        osDelay(10);  // 10ms cycle (100Hz control loop)
    }
}

void task_class::runSensorTask()
{
    // 센서 초기화 대기
    osDelay(200);
    
    // IMU 센서 초기화
    bool imu_initialized = false;
    if (pImuSensor != nullptr) {
        if (pImuSensor->init()) {
            imu_initialized = true;
            // WHO_AM_I 확인
            uint8_t whoami = pImuSensor->getWhoAmI();
            // whoami == 0x67 이면 정상
        }
    }
    
    // FET Temperature 센서 초기화 (ADC1)
    pFetTemp = new sensor_fettemp_class(&hadc1);
    
    ImuData_t imuData;
    ImuRawData_t imuRawData;
    EncoderData_t encoderData;
    
    // Main loop - 50ms cycle (20Hz)
    for(;;)
    {
        // IMU 센서 읽기
        if (imu_initialized && pImuSensor != nullptr) {
            if (pImuSensor->readImuData(&imuRawData)) {
                // Convert ImuRawData_t to ImuData_t for Queue
                imuData.accel_x = imuRawData.accel_x;
                imuData.accel_y = imuRawData.accel_y;
                imuData.accel_z = imuRawData.accel_z;
                imuData.gyro_x = imuRawData.gyro_x;
                imuData.gyro_y = imuRawData.gyro_y;
                imuData.gyro_z = imuRawData.gyro_z;
            }
        } else {
            // IMU 초기화 실패 시 기본값
            imuData.accel_x = 0.0f;
            imuData.accel_y = 0.0f;
            imuData.accel_z = 9.81f;
            imuData.gyro_x = 0.0f;
            imuData.gyro_y = 0.0f;
            imuData.gyro_z = 0.0f;
        }
        
        // Queue로 IMU 데이터 전송 (MicroRosTask로)
        osMessageQueuePut(imuDataQueueHandle, &imuData, 0, 0);
        
        // Encoder 읽기
        // TODO: Read actual encoder values
        encoderData.left_encoder = 0;
        encoderData.right_encoder = 0;
        
        // Queue로 Encoder 데이터 전송 (MicroRosTask로)
        osMessageQueuePut(encoderDataQueueHandle, &encoderData, 0, 0);
        
        // FET Temperature 읽기
        if (pFetTemp != nullptr) {
            pFetTemp->update_temperature();
            pFetTemp->publish_temperature();
            
            FetTempData_t fetTempData;
            fetTempData.temperature = pFetTemp->get_temperature();
            fetTempData.adc_raw = pFetTemp->get_adc_raw();
            
            // Queue로 Temperature 데이터 전송 (MicroRosTask로)
            osMessageQueuePut(fetTempQueueHandle, &fetTempData, 0, 0);
        }
        
        osDelay(50);  // 50ms cycle (20Hz sensor reading)
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
