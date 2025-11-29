/*
 * pwm_motor_class.cpp
 *
 *  Created on: Nov 25, 2025
 *      Author: thpark
 */

#include <pwm_dcmotor_class/pwm_dcmotor_class.h>

pwm_dcmotor_class::pwm_dcmotor_class() : hadc(nullptr), htim_pwm(nullptr)
{
	// Initialize all motor speeds to 0
	for (int i = 0; i < MOTOR_MAX; i++) {
		motor_speeds[i] = 0;
		overcurrent_flag[i] = false;
		overcurrent_count[i] = 0;
	}
}

pwm_dcmotor_class::pwm_dcmotor_class(ADC_HandleTypeDef *adc_handle) : hadc(adc_handle), htim_pwm(nullptr)
{
	// Initialize all motor speeds to 0
	for (int i = 0; i < MOTOR_MAX; i++) {
		motor_speeds[i] = 0;
		overcurrent_flag[i] = false;
		overcurrent_count[i] = 0;
	}
}

pwm_dcmotor_class::pwm_dcmotor_class(ADC_HandleTypeDef *adc_handle, TIM_HandleTypeDef *tim_handle) 
	: hadc(adc_handle), htim_pwm(tim_handle)
{
	// Initialize all motor speeds to 0
	for (int i = 0; i < MOTOR_MAX; i++) {
		motor_speeds[i] = 0;
		overcurrent_flag[i] = false;
		overcurrent_count[i] = 0;
	}
	
	// Start PWM channels with complementary outputs if timer is provided
	if (htim_pwm != nullptr) {
		// Motor 1: CH1 and CH1N
		HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(htim_pwm, TIM_CHANNEL_1);
		
		// Motor 2: CH2 and CH2N
		HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(htim_pwm, TIM_CHANNEL_2);
		
		// Initialize to stopped state
		__HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_2, 0);
	}
}

pwm_dcmotor_class::~pwm_dcmotor_class()
{
	// Stop all motors
	for (int i = 0; i < MOTOR_MAX; i++) {
		motor_speeds[i] = 0;
	}
}

void pwm_dcmotor_class::PwmMotor_Set_Speed(motor_id_t motor_id, int speed)
{
	// Validate motor ID
	if (motor_id >= MOTOR_MAX) {
		return;
	}
	
	// Clamp speed to -100 ~ +100 range
	if (speed > 100) {
		speed = 100;
	} else if (speed < -100) {
		speed = -100;
	}
	
	// Set motor speed
	motor_speeds[motor_id] = speed;
}

int pwm_dcmotor_class::PwmMotor_Get_Speed(motor_id_t motor_id)
{
	// Validate motor ID
	if (motor_id >= MOTOR_MAX) {
		return 0;
	}
	
	return motor_speeds[motor_id];
}

void pwm_dcmotor_class::pwm_update()
{
	// Check for overcurrent condition first
	check_overcurrent();
	
	// Apply PWM to all motors
	for (int i = 0; i < MOTOR_MAX; i++) {
		apply_motor_pwm((motor_id_t)i, motor_speeds[i]);
	}
}

// Private: Apply PWM to specific motor using complementary outputs
void pwm_dcmotor_class::apply_motor_pwm(motor_id_t motor_id, int speed)
{
	if (htim_pwm == nullptr) {
		return;  // Timer not initialized
	}
	
	// Convert speed (-100 to +100) to PWM duty (0 to ARR)
	// Timer1 ARR = 8399 -> 10kHz PWM with 84MHz clock
	uint32_t arr = htim_pwm->Init.Period;
	uint32_t duty = 0;
	bool forward = true;
	
	if (speed > 0) {
		forward = true;
		duty = (uint32_t)((speed * arr) / 100);
	} else if (speed < 0) {
		forward = false;
		duty = (uint32_t)((-speed * arr) / 100);
	} else {
		duty = 0;  // Stop (both outputs LOW)
	}
	
	// Clamp duty cycle
	if (duty > arr) {
		duty = arr;
	}
	
	// Apply to motors using complementary outputs (CHx/CHxN)
	// Forward: CHx=PWM, CHxN=0
	// Reverse: CHx=0, CHxN=PWM
	switch (motor_id) {
		case MOTOR_1:
			if (forward) {
				// Forward: CH1=PWM, CH1N=0
				__HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, duty);
			} else {
				// Reverse: CH1=0, CH1N=PWM (inverted by complementary output)
				__HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, arr - duty);
			}
			break;
			
		case MOTOR_2:
			if (forward) {
				// Forward: CH2=PWM, CH2N=0
				__HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_2, duty);
			} else {
				// Reverse: CH2=0, CH2N=PWM (inverted by complementary output)
				__HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_2, arr - duty);
			}
			break;
			
		case MOTOR_3:
		case MOTOR_4:
			// Motors 3 and 4 not implemented yet
			break;
			
		default:
			break;
	}
}

// Private: Read ADC channel
uint16_t pwm_dcmotor_class::read_adc_channel(uint32_t channel)
{
	if (hadc == nullptr) {
		return 0;
	}
	
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		return 0;
	}
	
	// Start ADC conversion
	HAL_ADC_Start(hadc);
	
	// Wait for conversion to complete (timeout: 100ms)
	if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK) {
		uint16_t adc_value = HAL_ADC_GetValue(hadc);
		HAL_ADC_Stop(hadc);
		return adc_value;
	}
	
	HAL_ADC_Stop(hadc);
	return 0;
}

// Private: Convert ADC value to current (mA)
float pwm_dcmotor_class::convert_adc_to_current(uint16_t adc_value)
{
	// Convert ADC value to voltage
	float voltage = (float)adc_value * ADC_VREF / ADC_RESOLUTION;
	
	// Convert voltage to current using gain (mA)
	float current_mA = (voltage / CURRENT_SENSE_GAIN) * 1000.0f;
	
	return current_mA;
}

// Public: Check overcurrent for all motors
void pwm_dcmotor_class::check_overcurrent()
{
	if (hadc == nullptr) {
		return;  // ADC not initialized
	}
	
	// BR_SO1: Motor 1 current (PB1 = ADC1_IN9)
	uint16_t adc_motor1 = read_adc_channel(ADC_CHANNEL_9);
	float current1_mA = convert_adc_to_current(adc_motor1);
	
	if (current1_mA > CURRENT_THRESHOLD_mA) {
		overcurrent_flag[MOTOR_1] = true;
		overcurrent_count[MOTOR_1]++;
		// Emergency stop for motor 1
		motor_speeds[MOTOR_1] = 0;
	}
	
	// BR_SO2: Motor 2 current (PB0 = ADC1_IN8)
	uint16_t adc_motor2 = read_adc_channel(ADC_CHANNEL_8);
	float current2_mA = convert_adc_to_current(adc_motor2);
	
	if (current2_mA > CURRENT_THRESHOLD_mA) {
		overcurrent_flag[MOTOR_2] = true;
		overcurrent_count[MOTOR_2]++;
		// Emergency stop for motor 2
		motor_speeds[MOTOR_2] = 0;
	}
}

// Public: Check if specific motor has overcurrent
bool pwm_dcmotor_class::is_overcurrent(motor_id_t motor_id)
{
	if (motor_id >= MOTOR_MAX) {
		return false;
	}
	return overcurrent_flag[motor_id];
}

// Public: Get current for specific motor (mA)
float pwm_dcmotor_class::get_motor_current(motor_id_t motor_id)
{
	if (hadc == nullptr || motor_id >= MOTOR_MAX) {
		return 0.0f;
	}
	
	uint32_t channel;
	if (motor_id == MOTOR_1) {
		channel = ADC_CHANNEL_9;  // BR_SO1 (PB1)
	} else if (motor_id == MOTOR_2) {
		channel = ADC_CHANNEL_8;  // BR_SO2 (PB0)
	} else {
		return 0.0f;  // Only MOTOR_1 and MOTOR_2 have current sensing
	}
	
	uint16_t adc_value = read_adc_channel(channel);
	return convert_adc_to_current(adc_value);
}

// Public: Clear overcurrent flag
void pwm_dcmotor_class::clear_overcurrent_flag(motor_id_t motor_id)
{
	if (motor_id >= MOTOR_MAX) {
		return;
	}
	overcurrent_flag[motor_id] = false;
}

// Public: Get overcurrent event count
uint32_t pwm_dcmotor_class::get_overcurrent_count(motor_id_t motor_id)
{
	if (motor_id >= MOTOR_MAX) {
		return 0;
	}
	return overcurrent_count[motor_id];
}

// Public: Reset overcurrent counter
void pwm_dcmotor_class::reset_overcurrent_count(motor_id_t motor_id)
{
	if (motor_id >= MOTOR_MAX) {
		return;
	}
	overcurrent_count[motor_id] = 0;
}
