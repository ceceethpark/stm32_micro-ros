# STM32F405 micro-ROS Robot Control System

STM32F405RGTx 기반 micro-ROS 로봇 제어 시스템 프로젝트입니다. FreeRTOS와 micro-ROS를 활용하여 듀얼 모터 제어, 센서 데이터 수집, ROS2 통신을 지원합니다.

## 주요 기능

### 하드웨어 구성
- **MCU**: STM32F405RGTx (ARM Cortex-M4, 168MHz)
- **RTOS**: FreeRTOS
- **통신**: USB CDC (디버그), UART3 (micro-ROS Agent)

### 모터 제어
- **PWM 제어**: Timer1 complementary outputs (CH1/CH1N, CH2/CH2N)
  - H-bridge 모터 드라이버 지원
  - 주파수: 10kHz
  - Dead-time: 58 cycles
  - 듀얼 모터 독립 제어
- **과전류 보호**: ADC 기반 실시간 전류 모니터링 (PB0, PB1)

### 엔코더
- **Timer3**: Motor1 엔코더 (PC6/PC7)
- **Timer4**: Motor2 엔코더
- **사양**: PPR 1000, CPR 4000
- **기능**: RPM 측정 및 실시간 퍼블리싱

### 센서
- **IMU**: SPI1 인터페이스 (ICM-42688-P)
  - 가속도계, 자이로스코프
  - CS: PA4
- **온도 센서**: NTC 10K (Beta=1360)
  - ADC1 CH3 (PA3)
  - FET 과열 보호용

### LED 제어
- **LED_GREEN** (PC4): 1Hz 하트비트
- **LED_RED** (PC5): 상태 표시
- Blink 기능 지원

## 소프트웨어 아키텍처

### FreeRTOS 태스크
1. **MicroRosTask** (50ms)
   - ROS2 통신 처리
   - cmd_vel 구독
   - 센서 데이터 퍼블리시

2. **ControlTask** (10ms, 100Hz)
   - 모터 PWM 제어
   - 엔코더 RPM 업데이트
   - LED 업데이트

3. **SensorTask** (50ms, 20Hz)
   - IMU 데이터 읽기
   - FET 온도 모니터링
   - 센서 데이터 큐 전송

### 클래스 구조
```
class/
├── DataClass/              # 데이터 관리 클래스
├── microros_class/         # micro-ROS 통신
├── pwm_dcmotor_class/      # PWM 모터 제어
├── sensor_encoder_class/   # 듀얼 엔코더
├── sensor_imu_class/       # IMU 센서
├── sensor_fettemp_class/   # 온도 센서
├── led_class/              # LED 제어
└── task_class/             # FreeRTOS 태스크 관리
```

### 큐 기반 통신
- `cmdVelQueueHandle`: cmd_vel 데이터 (MicroRosTask → ControlTask)
- `imuDataQueueHandle`: IMU 데이터 (SensorTask → MicroRosTask)
- `encoderDataQueueHandle`: 엔코더 데이터 (SensorTask → MicroRosTask)
- `fetTempQueueHandle`: 온도 데이터 (SensorTask → MicroRosTask)

## 핀 맵핑

### Timer1 (PWM)
- CH1 (PA8): Motor1 Forward
- CH1N (PB13): Motor1 Backward
- CH2 (PA9): Motor2 Forward
- CH2N (PB14): Motor2 Backward

### Timer3/4 (Encoder)
- Timer3 CH1/CH2 (PC6/PC7): Motor1 Encoder
- Timer4 CH1/CH2: Motor2 Encoder

### ADC1
- CH3 (PA3): FET Temperature (NTC 10K)
- CH8 (PB0): Motor2 Current Sense
- CH9 (PB1): Motor1 Current Sense

### SPI1
- SCK (PA5): IMU Clock
- MISO (PA6): IMU Data In
- MOSI (PA7): IMU Data Out
- CS (PA4): IMU Chip Select

### UART3
- TX (PB10): micro-ROS Agent
- RX (PB11): micro-ROS Agent

### GPIO
- PC4: LED_GREEN
- PC5: LED_RED

## 빌드 환경

### 요구사항
- STM32CubeMX
- ARM GCC 10.3.1
- micro-ROS static library
- make

### 빌드 방법
```powershell
# 1. micro-ROS 라이브러리 빌드
cd micro_ros_stm32cubemx_utils
.\build_microros.ps1

# 2. 프로젝트 빌드
cd Debug
make -j8
```

### 빌드 결과
```
text      data     bss     dec     hex filename
129956     960   64752  195668   2fc54 stm32_micro-ros.elf
```

## micro-ROS 설정

### Agent 연결
```bash
# UART3 (115200 baud)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### ROS2 토픽
- Subscribe: `/cmd_vel` (geometry_msgs/Twist)
- Publish: `/debug_counter` (std_msgs/Int32)
- Publish: `/imu/data` (sensor_msgs/Imu) - TODO
- Publish: `/encoder/rpm` (custom) - TODO

## 디버그

### USB CDC Printf
- USB 가상 COM 포트를 통한 printf 출력
- 초기화 메시지 및 센서 상태 확인

```c
printf("===================================\n");
printf("STM32F405 micro-ROS System Started\n");
```

## 개발 이력

### 2025.11
- 초기 프로젝트 설정
- 링커 에러 수정
- ADC 과전류 보호 구현
- Timer3 엔코더 추가

### 2025.11 중순
- sensor_encoder_class 리팩토링
- Timer4 추가로 듀얼 모터 엔코더 지원
- USB CDC printf 리다이렉션 (UART3는 micro-ros 전용)

### 2025.11 후반
- sensor_fettemp_class 추가 (NTC 온도 센서)
- pwm_dcmotor_class에 실제 PWM 모터 구동 구현
- Timer1 complementary outputs으로 H-bridge 제어

### 2025.11 말
- led_class 추가 (on/off/blink 기능)
- LED를 ControlTask에 통합 (10ms 업데이트)
- 1Hz 그린 LED 하트비트

### 2025.12
- Git 저장소 초기화
- GitHub 업로드 (ceceethpark/stm32_micro-ros)

## 라이선스

이 프로젝트는 다음 오픈소스 라이브러리를 사용합니다:
- STM32 HAL Driver (BSD-3-Clause)
- FreeRTOS (MIT)
- micro-ROS (Apache 2.0)
- CMSIS (Apache 2.0)

## 기여

이슈 및 풀 리퀘스트는 환영합니다.

## 작성자

thpark (ceceethpark)
