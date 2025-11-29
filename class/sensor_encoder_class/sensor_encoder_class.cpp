/*
 * sensor_encoder_class.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#include <sensor_encoder_class/sensor_encoder_class.h>

sensor_encoder_class::sensor_encoder_class() 
	: htim_encoder_motor1(nullptr), htim_encoder_motor2(nullptr), encoder_ppr(ENCODER_PPR)
{
	// Initialize encoder values
	for (int i = 0; i < ENCODER_MAX; i++) {
		prev_encoder_count[i] = 0;
		prev_time_ms[i] = 0;
		rpm_values[i] = 0.0f;
	}
}

sensor_encoder_class::sensor_encoder_class(TIM_HandleTypeDef *tim_motor1, TIM_HandleTypeDef *tim_motor2, uint16_t ppr) 
	: htim_encoder_motor1(tim_motor1), htim_encoder_motor2(tim_motor2), encoder_ppr(ppr)
{
	// Initialize encoder values
	for (int i = 0; i < ENCODER_MAX; i++) {
		prev_encoder_count[i] = 0;
		prev_time_ms[i] = HAL_GetTick();
		rpm_values[i] = 0.0f;
	}
}

sensor_encoder_class::~sensor_encoder_class()
{
	// Cleanup
}

// Private: Get encoder count from Timer3 (motor1) or Timer4 (motor2)
int32_t sensor_encoder_class::get_encoder_count(encoder_motor_id_t motor_id)
{
	TIM_HandleTypeDef *htim = nullptr;
	
	if (motor_id == ENCODER_MOTOR_1) {
		htim = htim_encoder_motor1;
	} else if (motor_id == ENCODER_MOTOR_2) {
		htim = htim_encoder_motor2;
	}
	
	if (htim == nullptr) {
		return 0;
	}
	
	// Read timer counter (16-bit, quadrature mode)
	// Counter increases on CW rotation, decreases on CCW
	return (int32_t)__HAL_TIM_GET_COUNTER(htim);
}

// Public: Update RPM calculation (call periodically, e.g., every 100ms)
void sensor_encoder_class::update_rpm()
{
	uint32_t current_time_ms = HAL_GetTick();
	
	// Calculate RPM for each motor
	for (int i = 0; i < ENCODER_MAX; i++) {
		encoder_motor_id_t motor_id = (encoder_motor_id_t)i;
		
		// Get current encoder count
		int32_t current_count = get_encoder_count(motor_id);
		
		// Calculate time difference
		uint32_t time_diff_ms = current_time_ms - prev_time_ms[motor_id];
		
		if (time_diff_ms >= RPM_UPDATE_INTERVAL_MS) {
			// Calculate count difference (handle 16-bit overflow)
			int32_t count_diff = current_count - prev_encoder_count[motor_id];
			
			// Handle 16-bit timer overflow (65536 counts)
			if (count_diff > 32768) {
				count_diff -= 65536;
			} else if (count_diff < -32768) {
				count_diff += 65536;
			}
			
			// Calculate RPM: (counts * 60000) / (counts_per_rev * time_ms)
			// Positive RPM = CW rotation, Negative RPM = CCW rotation
			float rpm = (float)(count_diff * 60000.0f) / (float)(ENCODER_CPR * time_diff_ms);
			
			rpm_values[motor_id] = rpm;
			
			// Update previous values
			prev_encoder_count[motor_id] = current_count;
			prev_time_ms[motor_id] = current_time_ms;
		}
	}
}

// Public: Get current RPM value
float sensor_encoder_class::get_rpm(encoder_motor_id_t motor_id)
{
	if (motor_id >= ENCODER_MAX) {
		return 0.0f;
	}
	return rpm_values[motor_id];
}

// Public: Reset encoder count and RPM
void sensor_encoder_class::reset_encoder(encoder_motor_id_t motor_id)
{
	if (motor_id >= ENCODER_MAX) {
		return;
	}
	
	prev_encoder_count[motor_id] = get_encoder_count(motor_id);
	prev_time_ms[motor_id] = HAL_GetTick();
	rpm_values[motor_id] = 0.0f;
}

// Public: Publish RPM values (placeholder for ROS publishing)
void sensor_encoder_class::publish_rpm()
{
	// Prepare RPM data structure
	rpm_data_t rpm_data;
	rpm_data.rpm_motor1 = rpm_values[ENCODER_MOTOR_1];
	rpm_data.rpm_motor2 = rpm_values[ENCODER_MOTOR_2];
	rpm_data.timestamp_ms = HAL_GetTick();
	
	// TODO: Implement actual ROS publishing or queue sending
	// Example: Send to ROS topic or task queue
	// microros_publish_rpm(&rpm_data);
	// or: xQueueSend(rpm_queue, &rpm_data, 0);
	
	// For debugging: print to serial (optional)
	#ifdef DEBUG_RPM
	printf("RPM - Motor1: %.2f (Raw: %ld), Motor2: %.2f (Raw: %ld)\r\n", 
		rpm_data.rpm_motor1, get_raw_count(ENCODER_MOTOR_1),
		rpm_data.rpm_motor2, get_raw_count(ENCODER_MOTOR_2));
	#endif
}

// Public: Get raw encoder count
int32_t sensor_encoder_class::get_raw_count(encoder_motor_id_t motor_id)
{
	if (motor_id >= ENCODER_MAX) {
		return 0;
	}
	return get_encoder_count(motor_id);
}

