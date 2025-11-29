# STM32F405RGTx 프로젝트에 micro-ROS 추가 가이드

## 현재 상태
- STM32F405RGTx 기반 프로젝트
- FreeRTOS 사용 중
- UART1 115200 baud 설정됨
- STM32CubeIDE 프로젝트

## 단계별 설정 방법

### 1. STM32CubeMX에서 프로젝트 설정 변경

#### 1.1 Makefile 툴체인으로 변경
1. `stm32_micro-ros.ioc` 파일을 STM32CubeMX에서 엽니다
2. `Project Manager` → `Project` 탭으로 이동
3. `Toolchain/IDE`를 **Makefile**로 변경
4. 코드를 재생성합니다 (Generate Code 버튼)

#### 1.2 FreeRTOS 스택 크기 증가
1. `Middleware` → `FreeRTOS` → `Tasks and Queues`로 이동
2. `defaultTask`의 Stack Size를 최소 **10240 bytes (2560 words)** 이상으로 변경
3. 또는 `Core/Src/freertos.c`에서 직접 수정:
   ```c
   const osThreadAttr_t defaultTask_attributes = {
     .name = "defaultTask",
     .stack_size = 10240,  // 기존 128 * 4에서 변경
     .priority = (osPriority_t) osPriorityNormal,
   };
   ```

#### 1.3 UART DMA 설정 (권장)
1. `Connectivity` → `USART1` 선택
2. `DMA Settings` 탭:
   - `Add` 버튼 클릭
   - USART1_RX: Mode를 **Circular**로 설정, Priority를 **Very High**로 설정
   - USART1_TX: Priority를 **Very High**로 설정
3. `NVIC Settings` 탭:
   - "USART1 global interrupt" 체크박스 활성화

### 2. micro-ROS 라이브러리 준비

#### 방법 A: Docker 사용 (권장)
Docker Desktop을 설치한 후:

```powershell
cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -v ${PWD}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
```

#### 방법 B: WSL2에서 빌드
WSL2에 Ubuntu가 설치되어 있다면:

```bash
cd /mnt/c/Users/thpark/SynologyDrive/wspace/stm32_micro-ros
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
```

#### 방법 C: 사전 빌드된 라이브러리 다운로드
1. [micro-ROS releases](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/releases)에서 사전 빌드된 라이브러리 다운로드
2. `micro_ros_stm32cubemx_utils/microros_static_library/` 폴더에 압축 해제

### 3. Makefile 수정

생성된 `Makefile`을 열어 **"build the application" 섹션 이전에** 다음 내용을 추가합니다:

```makefile
#######################################
# micro-ROS addons
#######################################
LDFLAGS += micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
C_INCLUDES += -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include

# Add micro-ROS utils
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_time.c

# Set here the custom transport implementation
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c

print_cflags:
	@echo $(CFLAGS)
```

### 4. 메인 코드 작성

`Core/Src/main.c`의 USER CODE 섹션에 다음을 추가:

```c
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
// micro-ROS 초기화는 FreeRTOS task에서 수행
/* USER CODE END 2 */
```

### 5. FreeRTOS Task 구현

`StartDefaultTask` 함수를 다음과 같이 수정:

```c
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  
  // micro-ROS 전송 계층 설정
  rmw_uros_set_custom_transport(
    true, // framing enable
    (void *) &huart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read
  );
  
  // micro-ROS 초기화
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  
  // Agent와 연결될 때까지 대기
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  
  if (ret == RCL_RET_OK) {
    // 노드 생성
    rcl_node_t node;
    rclc_node_init_default(&node, "stm32_node", "", &support);
    
    // Publisher 생성
    rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "stm32_publisher"
    );
    
    msg.data = 0;
    
    // Executor 설정
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    // 메인 루프
    for(;;) {
      rcl_publish(&publisher, &msg, NULL);
      msg.data++;
      osDelay(1000); // 1초마다 퍼블리시
    }
  }
  
  /* USER CODE END 5 */
}
```

### 6. 빌드 및 플래싱

```powershell
cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
make -j8
# STM32CubeProgrammer 또는 ST-Link를 사용하여 플래싱
```

### 7. micro-ROS Agent 실행

PC에서 ROS 2 환경에서:

```bash
# UART 연결 (Windows에서는 COMx)
ros2 run micro_ros_agent micro_ros_agent serial --dev COM3 -b 115200

# 토픽 확인
ros2 topic list
ros2 topic echo /stm32_publisher
```

## 트러블슈팅

### 메모리 부족
- FreeRTOS heap 크기 증가: `FreeRTOSConfig.h`에서 `configTOTAL_HEAP_SIZE` 증가
- Task 스택 크기 확인

### Agent 연결 실패
- UART 설정 확인 (보레이트, 포트 번호)
- DMA 설정 확인
- Agent가 올바른 시리얼 포트에 연결되었는지 확인

### 빌드 오류
- ARM GCC 툴체인이 설치되어 있는지 확인
- Makefile 경로가 올바른지 확인
- `colcon.meta` 파일을 수정하여 메모리 설정 조정

## 참고 자료
- [micro-ROS 공식 문서](https://micro.ros.org/)
- [micro-ROS STM32 GitHub](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
- [샘플 코드](micro_ros_stm32cubemx_utils/sample_main.c)
