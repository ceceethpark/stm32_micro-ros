/*
 * led_class.h
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#ifndef LED_CLASS_LED_CLASS_H_
#define LED_CLASS_LED_CLASS_H_

extern "C" {
#include "main.h"
}

// LED ID enumeration
typedef enum {
	LED_GREEN = 0,
	LED_RED = 1,
	LED_MAX = 2
} led_id_t;

class led_class
{
private:
	bool led_states[LED_MAX];  // Current LED states (true=ON, false=OFF)
	bool blink_enabled[LED_MAX];  // Blink enable flags
	uint32_t blink_period[LED_MAX];  // Blink period in ms
	uint32_t last_toggle_time[LED_MAX];  // Last toggle timestamp
	
	// Private helper methods
	void set_gpio_state(led_id_t led_id, bool state);
	
public:
	led_class();
	virtual ~led_class();
	
	// LED control methods
	void led_on(led_id_t led_id);
	void led_off(led_id_t led_id);
	void led_toggle(led_id_t led_id);
	void led_blink(led_id_t led_id, uint32_t period_ms);
	void led_blink_stop(led_id_t led_id);
	
	// Update method (call periodically from task)
	void update();
	
	// Get LED state
	bool get_state(led_id_t led_id);
};

#endif /* LED_CLASS_LED_CLASS_H_ */
