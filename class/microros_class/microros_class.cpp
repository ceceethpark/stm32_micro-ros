/*
 * microros.cpp
 *
 *  Created on: Nov 25, 2025
 *      Author: thpark
 */
#include "microros_class.h"
#include "extern.h"

// External transport functions (defined in Core/Src/microros_transports/dma_transport.c)
extern "C" {
    bool cubemx_transport_open(struct uxrCustomTransport * transport);
    bool cubemx_transport_close(struct uxrCustomTransport * transport);
    size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
    size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
    
    void * microros_allocate(size_t size, void * state);
    void microros_deallocate(void * pointer, void * state);
    void * microros_reallocate(void * pointer, size_t size, void * state);
    void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
}

microros_class::microros_class(UART_HandleTypeDef* uart, const char* node_name, 
                   const char* pub_topic, const char* sub_topic, const char* cmd_vel_topic)
    : huart(uart), node_name(node_name), 
      pub_topic_name(pub_topic), sub_topic_name(sub_topic),
      cmd_vel_topic_name(cmd_vel_topic),
      initialized(false), 
      has_subscriber(sub_topic != nullptr),
      has_cmd_vel_subscriber(cmd_vel_topic != nullptr),
      user_callback(nullptr),
      cmd_vel_callback(nullptr) {
    pub_msg.data = 0;
    sub_msg.data = 0;
    
    // Initialize cmd_vel message
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;
}

microros_class::~microros_class() {
    // Cleanup if needed
}

bool microros_class::init() {
    printf("[MICROROS] Setting up UART transport...\r\n");
    
    // Set custom transport for UART
    rmw_uros_set_custom_transport(
        true,
        (void *) huart,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);
    
    printf("[MICROROS] Transport configured\r\n");

    // Set FreeRTOS allocator
    printf("[MICROROS] Setting up FreeRTOS allocator...\r\n");
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("[MICROROS] ERROR: Failed to set allocator\r\n");
        return false;
    }
    printf("[MICROROS] Allocator configured\r\n");

    // Get default allocator
    allocator = rcl_get_default_allocator();

    // Set client key to 20
    printf("[MICROROS] Setting client key to 20...\r\n");
    rmw_uros_options_set_client_key(20, (void *) huart);

    // Create init_options and wait for agent connection
    printf("[MICROROS] Waiting for ROS2 agent connection...\r\n");
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    
    if (ret != RCL_RET_OK) {
        printf("[MICROROS] ERROR: Agent connection failed (ret=%d)\r\n", ret);
        return false;
    }
    printf("[MICROROS] Agent connected!\r\n");

    // Create node
    printf("[MICROROS] Creating ROS2 node...\r\n");
    ret = rclc_node_init_default(&node, node_name, "", &support);
    if (ret != RCL_RET_OK) {
        printf("[MICROROS] ERROR: Node creation failed (ret=%d)\r\n", ret);
        return false;
    }
    printf("[MICROROS] Node created: %s\r\n", node_name);

    // Create executor (count subscriptions)
    int num_subscriptions = 0;
    if (has_subscriber) num_subscriptions++;
    if (has_cmd_vel_subscriber) num_subscriptions++;
    
    ret = rclc_executor_init(&executor, &support.context, 
                             num_subscriptions, &allocator);
    if (ret != RCL_RET_OK) {
        return false;
    }

    // Create publisher if topic name provided
    if (pub_topic_name != nullptr) {
        ret = rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            pub_topic_name);
        
        if (ret != RCL_RET_OK) {
            return false;
        }
    }

    // Create subscriber if topic name provided
    if (has_subscriber && sub_topic_name != nullptr) {
        ret = rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            sub_topic_name);
        
        if (ret != RCL_RET_OK) {
            return false;
        }

        // Add subscription to executor
        ret = rclc_executor_add_subscription(
            &executor, &subscriber, &sub_msg, 
            [](const void* msgin) {
                // Static callback wrapper - will call user callback if set
            }, 
            ON_NEW_DATA);
        
        if (ret != RCL_RET_OK) {
            return false;
        }
    }

    // Create cmd_vel subscriber if topic name provided
    if (has_cmd_vel_subscriber && cmd_vel_topic_name != nullptr) {
        ret = rclc_subscription_init_default(
            &cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            cmd_vel_topic_name);
        
        if (ret != RCL_RET_OK) {
            return false;
        }

        // Add cmd_vel subscription to executor
        ret = rclc_executor_add_subscription(
            &executor, &cmd_vel_subscriber, &cmd_vel_msg, 
            [](const void* msgin) {
                // Static callback wrapper for cmd_vel
            }, 
            ON_NEW_DATA);
        
        if (ret != RCL_RET_OK) {
            return false;
        }
    }

    initialized = true;
    return true;
}

bool microros_class::publish(int32_t data) {
    if (!initialized || pub_topic_name == nullptr) {
        return false;
    }
    
    pub_msg.data = data;
    rcl_ret_t ret = rcl_publish(&publisher, &pub_msg, NULL);
    
    return (ret == RCL_RET_OK);
}

void microros_class::setSubscriberCallback(SubscriberCallback callback) {
    user_callback = callback;
}

void microros_class::setCmdVelCallback(SubscriberCallback callback) {
    cmd_vel_callback = callback;
}

void microros_class::spin() {
    if (!initialized || (!has_subscriber && !has_cmd_vel_subscriber)) {
        return;
    }
    
    // Spin executor to process incoming messages
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    // Call Int32 subscriber callback if message received and callback is set
    if (has_subscriber && user_callback != nullptr) {
        user_callback(&sub_msg);
    }
    
    // Call cmd_vel callback if message received and callback is set
    if (has_cmd_vel_subscriber && cmd_vel_callback != nullptr) {
        cmd_vel_callback(&cmd_vel_msg);
    }
}

