/*
 * pwm_motor_class.h
 *
 *  Created on: Nov 25, 2025
 *      Author: thpark
 */

#ifndef PWM_MOTOR_CLASS_PWM_MOTOR_CLASS_H_
#define PWM_MOTOR_CLASS_PWM_MOTOR_CLASS_H_

#include "main.h"

// Motor ID enumeration
typedef enum {
	MOTOR_1 = 0,
	MOTOR_2 = 1,
	MOTOR_3 = 2,
	MOTOR_4 = 3,
	MOTOR_MAX
} motor_id_t;

// Overcurrent protection configuration
#define CURRENT_THRESHOLD_mA    3000   // 3A threshold
#define CURRENT_SENSE_GAIN      0.1f   // V/A (depends on hardware)
#define ADC_VREF                3.3f   // ADC reference voltage
#define ADC_RESOLUTION          4095   // 12-bit ADC

class pwm_dcmotor_class
{
private:
	int motor_speeds[MOTOR_MAX];  // Speed values for each motor
	bool overcurrent_flag[MOTOR_MAX];  // Overcurrent detection flags
	uint32_t overcurrent_count[MOTOR_MAX];  // Overcurrent event counter
	ADC_HandleTypeDef *hadc;  // ADC handle for current sensing
	TIM_HandleTypeDef *htim_pwm;  // Timer handle for PWM generation
	
	// Private methods
	uint16_t read_adc_channel(uint32_t channel);
	float convert_adc_to_current(uint16_t adc_value);
	void apply_motor_pwm(motor_id_t motor_id, int speed);
	
public:
	pwm_dcmotor_class();
	pwm_dcmotor_class(ADC_HandleTypeDef *adc_handle);
	pwm_dcmotor_class(ADC_HandleTypeDef *adc_handle, TIM_HandleTypeDef *tim_handle);
	virtual ~pwm_dcmotor_class();
	
	// Set motor speed (-100 to +100)
	void PwmMotor_Set_Speed(motor_id_t motor_id, int speed);
	
	// Get motor speed
	int PwmMotor_Get_Speed(motor_id_t motor_id);
	
	// PWM update method (called every 10ms)
	void pwm_update();
	
	// Overcurrent protection methods
	void check_overcurrent();  // Check current for all motors
	bool is_overcurrent(motor_id_t motor_id);  // Check if specific motor has overcurrent
	float get_motor_current(motor_id_t motor_id);  // Read current for specific motor
	void clear_overcurrent_flag(motor_id_t motor_id);  // Clear overcurrent flag
	uint32_t get_overcurrent_count(motor_id_t motor_id);  // Get overcurrent event count
	void reset_overcurrent_count(motor_id_t motor_id);  // Reset overcurrent counter
};

#endif /* PWM_MOTOR_CLASS_PWM_MOTOR_CLASS_H_ */
