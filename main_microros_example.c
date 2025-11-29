/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_microros_example.c
  * @brief          : micro-ROS example implementation for STM32F405RGTx
  ******************************************************************************
  * @attention
  * 
  * 이 파일은 main.c의 USER CODE 섹션에 추가할 예제 코드입니다.
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

// ============================================================================
// main.c의 USER CODE BEGIN Includes 섹션에 추가할 헤더
// ============================================================================
/*
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
*/

// ============================================================================
// main.c의 USER CODE BEGIN PV 섹션에 추가할 전역 변수
// ============================================================================
/*
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
*/

// ============================================================================
// main.c의 USER CODE BEGIN 4 섹션에 추가할 함수 선언
// ============================================================================
/*
// Transport layer function prototypes
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// Allocator function prototypes
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
*/

// ============================================================================
// freertos.c 파일 수정
// ============================================================================
// defaultTask_attributes의 stack_size를 최소 10240으로 변경:
/*
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 10240,  // 기존: 128 * 4
  .priority = (osPriority_t) osPriorityNormal,
};
*/

// ============================================================================
// StartDefaultTask 함수 구현 (main.c의 USER CODE BEGIN 5 섹션)
// ============================================================================
/*
void StartDefaultTask(void *argument)
{
  // USER CODE BEGIN 5
  
  // micro-ROS configuration
  // UART1을 micro-ROS 전송 계층으로 설정
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart1,  // UART1 핸들 사용
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  // FreeRTOS용 메모리 할당자 설정
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    // 에러 처리
    Error_Handler();
  }

  // micro-ROS 애플리케이션 초기화
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();

  // Support 초기화 (Agent와 연결 시도)
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  
  if (ret == RCL_RET_OK) {
    // 노드 생성
    rclc_node_init_default(&node, "stm32f405_node", "", &support);

    // Publisher 생성
    rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "stm32_counter");

    msg.data = 0;

    // 메인 루프
    for(;;)
    {
      // 메시지 퍼블리시
      ret = rcl_publish(&publisher, &msg, NULL);
      
      if (ret == RCL_RET_OK) {
        msg.data++;
      }
      
      // 100ms 마다 퍼블리시
      osDelay(100);
    }
  } else {
    // Agent 연결 실패 시 에러 처리
    Error_Handler();
  }
  
  // USER CODE END 5
}
*/

// ============================================================================
// FreeRTOSConfig.h 수정 사항
// ============================================================================
// configTOTAL_HEAP_SIZE를 충분히 크게 설정 (최소 32KB 권장):
/*
#define configTOTAL_HEAP_SIZE ((size_t)(32 * 1024))
*/

// ============================================================================
// 추가 예제: Subscriber 구현
// ============================================================================
/*
// 전역 변수에 추가
rcl_subscription_t subscriber;
std_msgs__msg__Int32 sub_msg;

// 콜백 함수
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // 수신한 데이터 처리
  // 예: LED 토글, 데이터 저장 등
}

// StartDefaultTask에서 subscriber 생성
void StartDefaultTask(void *argument)
{
  // ... 이전 코드 ...
  
  // Subscriber 생성
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stm32_input");

  // Executor 생성
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA);

  // 메인 루프 수정
  for(;;)
  {
    // 메시지 퍼블리시
    rcl_publish(&publisher, &msg, NULL);
    msg.data++;
    
    // Executor 실행 (subscription 처리)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    
    osDelay(100);
  }
}
*/

// ============================================================================
// PC에서 micro-ROS Agent 실행 방법
// ============================================================================
/*
Windows PowerShell 또는 CMD에서:

1. ROS 2 환경 설정
   > C:\ros2_humble\local_setup.bat

2. micro-ROS Agent 실행 (COM 포트 번호는 장치 관리자에서 확인)
   > ros2 run micro_ros_agent micro_ros_agent serial --dev COM3 -b 115200

3. 다른 터미널에서 토픽 확인
   > ros2 topic list
   > ros2 topic echo /stm32_counter

4. 데이터 전송 (Subscriber 구현 시)
   > ros2 topic pub /stm32_input std_msgs/msg/Int32 "data: 42"
*/

// ============================================================================
// 빌드 방법 (Makefile 생성 후)
// ============================================================================
/*
PowerShell에서:
> cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
> make -j8

또는 STM32CubeIDE에서 프로젝트 빌드
*/

// ============================================================================
// 트러블슈팅
// ============================================================================
/*
1. Agent 연결 실패
   - UART 설정 확인 (보레이트, DMA 설정)
   - COM 포트 번호 확인
   - Agent 실행 여부 확인

2. 메모리 부족
   - FreeRTOS heap 크기 증가
   - Task 스택 크기 증가
   - colcon.meta 파일에서 micro-ROS 메모리 설정 조정

3. 빌드 오류
   - ARM GCC 툴체인 설치 확인
   - Makefile 경로 확인
   - micro-ROS 라이브러리 빌드 확인
*/
