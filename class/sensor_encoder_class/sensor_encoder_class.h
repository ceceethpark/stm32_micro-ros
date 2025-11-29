/*
 * sensor_encoder_class.h
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#ifndef SENSOR_ENCODER_CLASS_SENSOR_ENCODER_CLASS_H_
#define SENSOR_ENCODER_CLASS_SENSOR_ENCODER_CLASS_H_

#include "main.h"

// Motor ID enumeration for encoder mapping
typedef enum {
	ENCODER_MOTOR_1 = 0,  // Timer3 encoder
	ENCODER_MOTOR_2 = 1,  // Timer4 encoder
	ENCODER_MAX = 2
} encoder_motor_id_t;

// Encoder configuration
#define ENCODER_PPR             1000   // Pulses Per Revolution (adjust based on encoder)
#define ENCODER_CPR             (ENCODER_PPR * 4)  // Counts Per Revolution (quadrature x4)
#define RPM_UPDATE_INTERVAL_MS  100    // RPM calculation interval in milliseconds

// RPM data structure for publishing (2 motors)
typedef struct {
	float rpm_motor1;  // Timer3 encoder
	float rpm_motor2;  // Timer4 encoder
	uint32_t timestamp_ms;
} rpm_data_t;

class sensor_encoder_class
{
private:
	// Encoder-related members
	TIM_HandleTypeDef *htim_encoder_motor1;  // Timer3 for motor1
	TIM_HandleTypeDef *htim_encoder_motor2;  // Timer4 for motor2
	int32_t prev_encoder_count[ENCODER_MAX];  // Previous encoder counts
	uint32_t prev_time_ms[ENCODER_MAX];  // Previous timestamp for RPM calculation
	float rpm_values[ENCODER_MAX];  // Calculated RPM values
	uint16_t encoder_ppr;  // Encoder pulses per revolution
	
	// Private methods
	int32_t get_encoder_count(encoder_motor_id_t motor_id);
	
public:
	sensor_encoder_class();
	sensor_encoder_class(TIM_HandleTypeDef *tim_motor1, TIM_HandleTypeDef *tim_motor2, uint16_t ppr = ENCODER_PPR);
	virtual ~sensor_encoder_class();
	
	// Encoder and RPM methods
	void update_rpm();  // Update RPM calculation (call periodically)
	float get_rpm(encoder_motor_id_t motor_id);  // Get current RPM value
	void reset_encoder(encoder_motor_id_t motor_id);  // Reset encoder count and RPM
	void publish_rpm();  // Publish RPM values (to ROS or other interface)
	
	// Direct encoder count access
	int32_t get_raw_count(encoder_motor_id_t motor_id);  // Get raw encoder count
};

#endif /* SENSOR_ENCODER_CLASS_SENSOR_ENCODER_CLASS_H_ */
