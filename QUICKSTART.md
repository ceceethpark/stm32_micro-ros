# STM32F405 + micro-ROS 빠른 시작 가이드

## 1. STM32CubeMX 설정

### 필수 변경 사항

1. **프로젝트 설정**
   - `stm32_micro-ros.ioc` 파일 열기
   - Project Manager → Project → Toolchain/IDE: **Makefile** 선택
   - Generate Code

2. **UART DMA 설정** (권장)
   - Connectivity → USART1
   - DMA Settings:
     - USART1_RX: Mode=**Circular**, Priority=**Very High**
     - USART1_TX: Priority=**Very High**
   - NVIC Settings: "USART1 global interrupt" 활성화

3. **FreeRTOS 스택 크기**
   - ✅ 이미 완료됨 (`main.c`에서 10240으로 설정)

## 2. micro-ROS 라이브러리 빌드

### 방법 A: Docker 사용 (권장)

```powershell
cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -v ${PWD}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
```

### 방법 B: WSL2 사용

```bash
cd /mnt/c/Users/thpark/SynologyDrive/wspace/stm32_micro-ros
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
```

## 3. Makefile 생성 및 수정

STM32CubeMX에서 Makefile이 생성되면, **"build the application" 섹션 앞에** 다음 추가:

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

## 4. 코드 통합

### ✅ 완료된 작업

- `Core/Src/main.c`에 micro-ROS 코드 추가됨
- FreeRTOS 스택 크기 10KB로 증가
- Publisher 예제 코드 구현

### 코드 요약

- **노드 이름**: `stm32f405_node`
- **토픽 이름**: `stm32_counter`
- **메시지 타입**: `std_msgs/msg/Int32`
- **퍼블리시 주기**: 100ms

## 5. 빌드

```powershell
cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
make -j8
```

빌드 산출물: `build/stm32_micro-ros.elf`, `.hex`, `.bin`

## 6. 플래싱

- STM32CubeProgrammer 사용
- ST-Link Utility 사용
- OpenOCD 사용

## 7. micro-ROS Agent 실행

### PC에서 (Windows)

```powershell
# ROS 2 환경 설정
C:\ros2_humble\local_setup.bat

# Agent 실행 (COM 포트는 장치 관리자에서 확인)
ros2 run micro_ros_agent micro_ros_agent serial --dev COM3 -b 115200
```

### 데이터 확인

```powershell
# 새 터미널
ros2 topic list
ros2 topic echo /stm32_counter
```

예상 출력:
```
data: 0
---
data: 1
---
data: 2
---
```

## 8. 추가 FreeRTOSConfig.h 설정

`Core/Inc/FreeRTOSConfig.h`에서 힙 크기 확인 및 증가:

```c
#define configTOTAL_HEAP_SIZE ((size_t)(32 * 1024))  // 32KB 권장
```

## 트러블슈팅

### 빌드 오류

1. **라이브러리 없음**
   - micro-ROS 라이브러리 빌드 확인
   - `micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a` 존재 확인

2. **링커 오류**
   - Makefile에 micro-ROS 설정 추가 확인
   - ARM GCC 툴체인 설치 확인

### Agent 연결 실패

1. **COM 포트 확인**
   - 장치 관리자에서 정확한 COM 포트 번호 확인
   - USB 드라이버 설치 확인

2. **보레이트 불일치**
   - STM32: 115200 (기본 설정됨)
   - Agent: `-b 115200` 옵션 확인

3. **DMA 설정**
   - STM32CubeMX에서 DMA 설정 재확인
   - 코드 재생성 후 빌드

### 메모리 부족

1. **스택 오버플로우**
   - Task 스택 크기 증가 (현재: 10240)
   - `configCHECK_FOR_STACK_OVERFLOW` 활성화

2. **힙 부족**
   - `configTOTAL_HEAP_SIZE` 증가
   - `colcon.meta` 파일에서 micro-ROS 메모리 설정 조정

## 다음 단계

1. **Subscriber 추가**: `main_microros_example.c` 참조
2. **Custom 메시지**: `colcon.meta`에 패키지 추가
3. **서비스/액션**: micro-ROS 고급 기능 활용

## 참고 자료

- [micro-ROS 공식 문서](https://micro.ros.org/)
- [GitHub 저장소](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
- 로컬 예제: `micro_ros_stm32cubemx_utils/sample_main.c`
- 통합 가이드: `MICROROS_SETUP_GUIDE.md`
- 예제 코드: `main_microros_example.c`
