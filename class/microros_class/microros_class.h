/*
 * microros.h
 *
 *  Created on: Nov 25, 2025
 *      Author: thpark
 */

#ifndef CLASS_MICROROS_MICROROS_H_
#define CLASS_MICROROS_MICROROS_H_

extern "C" {
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcutils/allocator.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include "main.h"
}

// Callback function type for subscriber
typedef void (*SubscriberCallback)(const void* msg);

class microros_class {
private:
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rclc_executor_t executor;
    
    // Publisher
    rcl_publisher_t publisher;
    std_msgs__msg__Int32 pub_msg;
    
    // Subscriber (Int32)
    rcl_subscription_t subscriber;
    std_msgs__msg__Int32 sub_msg;
    
    // Subscriber (Twist for cmd_vel)
    rcl_subscription_t cmd_vel_subscriber;
    geometry_msgs__msg__Twist cmd_vel_msg;
    
    UART_HandleTypeDef* huart;
    const char* node_name;
    const char* pub_topic_name;
    const char* sub_topic_name;
    const char* cmd_vel_topic_name;
    bool initialized;
    bool has_subscriber;
    bool has_cmd_vel_subscriber;
    
    // Callbacks
    SubscriberCallback user_callback;
    SubscriberCallback cmd_vel_callback;

public:
    // Constructor
    microros_class(UART_HandleTypeDef* uart, const char* node_name, 
             const char* pub_topic = nullptr, const char* sub_topic = nullptr,
             const char* cmd_vel_topic = nullptr);
    
    // Destructor
    ~microros_class();
    
    // Initialize micro-ROS (setup transport, allocator, node, publisher, subscriber)
    bool init();
    
    // Publish Int32 message
    bool publish(int32_t data);
    
    // Set Int32 subscriber callback
    void setSubscriberCallback(SubscriberCallback callback);
    
    // Set cmd_vel subscriber callback
    void setCmdVelCallback(SubscriberCallback callback);
    
    // Spin executor (call this periodically to receive messages)
    void spin();
    
    // Check if initialized
    bool isInitialized() const { return initialized; }
    
    // Get current publish message value
    int32_t getPubMessage() const { return pub_msg.data; }
    
    // Get current received Int32 message value
    int32_t getSubMessage() const { return sub_msg.data; }
    
    // Get cmd_vel linear/angular velocity
    float getCmdVelLinearX() const { return cmd_vel_msg.linear.x; }
    float getCmdVelLinearY() const { return cmd_vel_msg.linear.y; }
    float getCmdVelLinearZ() const { return cmd_vel_msg.linear.z; }
    float getCmdVelAngularX() const { return cmd_vel_msg.angular.x; }
    float getCmdVelAngularY() const { return cmd_vel_msg.angular.y; }
    float getCmdVelAngularZ() const { return cmd_vel_msg.angular.z; }
};

#endif /* CLASS_MICROROS_MICROROS_H_ */
