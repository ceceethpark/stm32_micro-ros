/*
 * led_class.cpp
 *
 *  Created on: Nov 26, 2025
 *      Author: thpark
 */

#include <led_class/led_class.h>

led_class::led_class()
{
	// Initialize all LEDs to OFF state
	for (int i = 0; i < LED_MAX; i++) {
		led_states[i] = false;
		blink_enabled[i] = false;
		blink_period[i] = 0;
		last_toggle_time[i] = 0;
	}
	
	// Turn off all LEDs
	led_off(LED_GREEN);
	led_off(LED_RED);
}

led_class::~led_class()
{
	// Turn off all LEDs on destruction
	led_off(LED_GREEN);
	led_off(LED_RED);
}

// Private: Set GPIO state for LED
void led_class::set_gpio_state(led_id_t led_id, bool state)
{
	GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
	
	switch (led_id) {
		case LED_GREEN:
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, pin_state);
			break;
			
		case LED_RED:
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, pin_state);
			break;
			
		default:
			break;
	}
}

// Public: Turn LED on
void led_class::led_on(led_id_t led_id)
{
	if (led_id >= LED_MAX) {
		return;
	}
	
	// Disable blink mode
	blink_enabled[led_id] = false;
	
	// Turn on LED
	led_states[led_id] = true;
	set_gpio_state(led_id, true);
}

// Public: Turn LED off
void led_class::led_off(led_id_t led_id)
{
	if (led_id >= LED_MAX) {
		return;
	}
	
	// Disable blink mode
	blink_enabled[led_id] = false;
	
	// Turn off LED
	led_states[led_id] = false;
	set_gpio_state(led_id, false);
}

// Public: Toggle LED state
void led_class::led_toggle(led_id_t led_id)
{
	if (led_id >= LED_MAX) {
		return;
	}
	
	// Toggle state
	led_states[led_id] = !led_states[led_id];
	set_gpio_state(led_id, led_states[led_id]);
}

// Public: Start LED blinking with specified period
void led_class::led_blink(led_id_t led_id, uint32_t period_ms)
{
	if (led_id >= LED_MAX || period_ms == 0) {
		return;
	}
	
	// Enable blink mode
	blink_enabled[led_id] = true;
	blink_period[led_id] = period_ms;
	last_toggle_time[led_id] = HAL_GetTick();
}

// Public: Stop LED blinking
void led_class::led_blink_stop(led_id_t led_id)
{
	if (led_id >= LED_MAX) {
		return;
	}
	
	// Disable blink mode
	blink_enabled[led_id] = false;
}

// Public: Update method (call periodically to handle blinking)
void led_class::update()
{
	uint32_t current_time = HAL_GetTick();
	
	for (int i = 0; i < LED_MAX; i++) {
		if (blink_enabled[i]) {
			// Check if it's time to toggle
			if ((current_time - last_toggle_time[i]) >= blink_period[i]) {
				led_toggle((led_id_t)i);
				last_toggle_time[i] = current_time;
			}
		}
	}
}

// Public: Get LED state
bool led_class::get_state(led_id_t led_id)
{
	if (led_id >= LED_MAX) {
		return false;
	}
	
	return led_states[led_id];
}