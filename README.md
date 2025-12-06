# STM32F405 micro-ROS Robot Control System

STM32F405RGT6 기반 micro-ROS 로봇 제어 시스템 프로젝트입니다. FreeRTOS와 micro-ROS를 활용하여 듀얼 모터 제어, 센서 데이터 수집, ROS2 통신을 지원합니다.

## 주요 기능

### 하드웨어 구성
- **MCU**: STM32F405RGT6 (LQFP64)
  - ARM Cortex-M4F, 168MHz
  - 1MB Flash, 128KB SRAM
  - FPU 지원
- **클럭**: 8MHz HSE → 168MHz SYSCLK, 48MHz USB
- **RTOS**: FreeRTOS (CMSIS-RTOS v2, 48KB Heap)
- **통신**: SWO (디버그 printf), UART2 (micro-ROS Agent)

### 모터 제어
- **PWM 제어**: Timer1 complementary outputs (CH1/CH1N, CH2/CH2N)
  - H-bridge 모터 드라이버 지원
  - 주파수: 10kHz
  - Dead-time: 58 cycles
  - 듀얼 모터 독립 제어
- **과전류 보호**: ADC 기반 실시간 전류 모니터링 (PB0, PB1)

### 엔코더
- **Timer3**: Motor1 엔코더 (PC6/PC7, CH1/CH2)
- **Timer4**: Motor2 엔코더 (CH1/CH2)
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

### Python 브리지 스크립트
```
stm32_micro-ros/
├── serial_bridge.py        # Windows: COM5 ↔ UDP 브리지
├── bridge_node.py          # WSL: ROS2 topics ↔ UDP (~/ros2_ws/src/stm32_bridge/)
└── udp_relay.py            # WSL: localhost UDP → Windows IP 릴레이
```

**실행 순서:**
1. WSL: `python3 ~/udp_relay.py` (백그라운드)
2. Windows: `python serial_bridge.py` (COM5 연결 필요)
3. WSL: `ros2 run stm32_bridge bridge_node`

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

### UART2 (micro-ROS)
- TX (PA2 / U6.16): micro-ROS Agent @ 921600 baud
- RX (PA3 / U6.17): micro-ROS Agent @ 921600 baud

### UART3 (Debug)
- TX (PB10): printf 디버그 출력 @ 115200 baud
- RX (PB11): Reserved

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
83252     1064   23432  107748   1a4e4 stm32_micro-ros.elf
```

**최적화 내역 (2025.12.06):**
- DMA 구현으로 코드 크기 감소
- 불필요한 micro-ROS 컴포넌트 제거
- Flash: 84316 bytes (82% 절약)
- RAM: 24496 bytes (62% 절약)

## micro-ROS 설정

### Agent 연결
```bash
# UART2 (921600 baud) - PA2/PA3
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 --key 20 -v6

# 또는 Docker 사용
docker run -it --rm --device=/dev/ttyUSB0 microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 921600
```

### ROS2 토픽
- Subscribe: `/cmd_vel` (geometry_msgs/Twist) - ✅ **구현 완료**
- Publish: `/imu/data` (JSON via UART2) - ✅ **구현 완료** @ 5Hz
- Publish: `/debug_counter` (std_msgs/Int32)
- Publish: `/encoder/rpm` (custom) - TODO

### 통신 아키텍처 (Bidirectional)
```
ROS2 Node (WSL)
    ↕ (ROS2 topics)
bridge_node.py (WSL)
    ↕ (UDP 8888/8889 via localhost)
udp_relay.py (WSL)
    ↕ (UDP 8889 to Windows IP:8889)
serial_bridge.py (Windows)
    ↕ (COM5 @ 921600 baud, JSON)
STM32 UART2 (DMA TX/RX)
    ↕
Bridge Task (FreeRTOS)
    → JSON parsing → cmd_vel Queue → Control Task → Motors
    ← Sensor Queue ← Sensor Task ← IMU
```

## 디버그

### SWO (Serial Wire Output) Printf
- **변경일**: 2025.12.04
- **변경 이유**: UART2를 micro-ROS agent와 겸용 불가, SWO로 분리
- **핀**: PB3 (JTDO/TRACESWO)
- ST-Link SWO 핀 연결 필요
- STM32CubeIDE SWV ITM Data Console에서 확인

**설정:**
- Debug Configuration → Serial Wire Viewer (SWV) 활성화
- Core Clock: 168 MHz
- SWO Clock: 2000 kHz
- ITM Stimulus Port 0 활성화

```c
// SWO printf 리다이렉션 (U6.55)
int _write(int32_t file, uint8_t *ptr, int32_t len) {
    for(int i = 0; i < len; i++) {
        ITM_SendChar(ptr[i]);
    }
    return len;
}
```

### 통신 포트 할당
- **UART2**: micro-ROS Agent 전용 (921600 baud)
  - TX: PA2 (U6.16)
  - RX: PA3 (U2.17)
- **UART3**: printf 디버그 출력 (115200 baud)
  - TX: PB10
  - RX: PB11
- **SWO (PB3)**: 사용 안 함 (이전: printf 디버그 출력)

## WSL2 환경 설정

### WSL 기본 명령어

**Windows PowerShell에서:**
```powershell
# WSL 실행
wsl

# 특정 배포판 실행
wsl -d Ubuntu-24.04

# WSL 완전 종료 (설정 적용 시 필요)
wsl --shutdown

# WSL 상태 확인
wsl --list --verbose

# 실행 중인 WSL 배포판 확인
wsl --list --running
```

**WSL(Linux) 내부에서:**
```bash
# 현재 터미널만 종료
exit

# 또는
logout

# Linux 시스템 종료 (WSL 전체 종료, 권장하지 않음)
sudo shutdown -h now
sudo poweroff
```

**참고:** `.wslconfig` 파일 변경 후에는 반드시 Windows PowerShell에서 `wsl --shutdown`을 실행해야 설정이 적용됩니다.

### 1. MicroXRCEAgent 설치

```bash
# WSL에서 소스 빌드
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 2. USB 장치 WSL 연결

Windows PowerShell (관리자 권한):

```powershell
# USB 장치 목록 확인
usbipd list

# USB 장치 바인딩 (최초 1회)
usbipd bind --busid 3-4

# WSL에 연결
usbipd attach --wsl --busid 3-4
```

WSL에서 확인:
```bash
ls /dev/ttyUSB0
```

### 3. Agent 실행

```bash
# Client key 20으로 실행
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 --key 20 -v6
```

### 4. ROS2 환경 설정

```bash
# ROS2 Jazzy 설치 (Ubuntu 24.04)
sudo apt install ros-jazzy-ros-base

# 환경 설정
source /opt/ros/jazzy/setup.bash

# Domain ID 설정 (다른 기기와 맞춰야 함)
export ROS_DOMAIN_ID=20
```

### 5. Cyclone DDS 설치 및 설정

```bash
# Cyclone DDS 설치
sudo apt install ros-jazzy-rmw-cyclonedds-cpp

# 환경 변수 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## WSL2 네트워크 설정 (로컬 네트워크 통신)

WSL2는 기본적으로 NAT 네트워크(172.x.x.x)를 사용하여 로컬 네트워크(192.168.0.x)와 직접 통신할 수 없습니다. 
로컬 네트워크의 다른 ROS2 기기와 통신하려면 다음 설정이 필요합니다.

### 방법 1: WSL Mirrored Networking (권장 - Windows 11만 해당)

**요구사항**: Windows 11 22H2 (Build 22621) 이상

**⚠️ 주의**: Windows 10에서는 이 방법을 사용할 수 없습니다. Windows 10 사용자는 **방법 2 (Cyclone DDS Peer 설정)**을 사용하세요.

**1. `.wslconfig` 파일 생성**

PowerShell에서:
```powershell
@"
[wsl2]
networkingMode=mirrored
dhcp=true
"@ | Out-File -FilePath "$env:USERPROFILE\.wslconfig" -Encoding utf8
```

또는 수동으로 `C:\Users\사용자명\.wslconfig` 파일 생성:
```ini
[wsl2]
networkingMode=mirrored
dhcp=true
```

**2. WSL 재시작**

PowerShell에서:
```powershell
wsl --shutdown
```

그 후 WSL을 다시 실행하면 WSL이 Windows와 같은 192.168.0.x 네트워크를 사용합니다.

**3. IP 확인**

```bash
ip addr show eth0
# 이제 192.168.0.x 대역 IP를 받습니다
```

### 방법 2: Hyper-V External Switch (Bridged Mode) - 실험적

**⚠️ 복잡하고 불안정할 수 있음 - Windows 10에서 방법 3 권장**

**1. Hyper-V 설치 및 External Switch 생성**

PowerShell (관리자 권한):
```powershell
# Hyper-V 기능 활성화
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V -All

# External Switch 생성 (물리적 네트워크 어댑터 선택)
New-VMSwitch -Name "WSLBridge" -NetAdapterName "이더넷" -AllowManagementOS $true
```

**2. `.wslconfig` 파일 생성**

PowerShell에서:
```powershell
@"
[wsl2]
networkingMode=bridged
vmSwitch=WSLBridge
dhcp=true
"@ | Out-File -FilePath "$env:USERPROFILE\.wslconfig" -Encoding utf8
```

**3. WSL 재시작 및 확인**

```powershell
wsl --shutdown
wsl
```

WSL 내부에서:
```bash
ip addr show eth0
# 192.168.0.x 대역 IP를 받아야 함
```

**⚠️ 주의사항:**
- vEthernet 어댑터는 DHCP 설정 불가 (정상 동작)
- Windows 네트워크 위치를 "개인 네트워크"로 설정해야 방화벽 문제 없음
  - 설정 → 네트워크 및 인터넷 → 이더넷/Wi-Fi → 네트워크 프로필: "개인"
- External Switch 생성 시 인터넷이 일시적으로 끊길 수 있음
- 불안정한 경우 **방법 3 (Cyclone DDS Peer)** 사용 권장

### 방법 3: Cyclone DDS Peer 설정 (가장 안정적)

**✅ Windows 10 권장 방법 - 가장 간단하고 안정적**

WSL IP가 172.x.x.x 대역인 경우, peer 설정으로 해결:

**WSL IP 확인:**
```bash
ip addr show eth0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1
# 예: 172.22.133.211
```

**WSL에서 cyclonedds.xml 생성:**

```bash
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain>
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer address="192.168.0.154"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

**다른 ROS2 기기(예: 192.168.0.154)에서도 WSL IP를 peer로 추가:**

```bash
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain>
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer address="172.22.133.211"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

### 영구 환경 변수 설정

`~/.bashrc`에 추가:

```bash
echo 'export ROS_DOMAIN_ID=20' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
```

## ROS2 명령어

### Topic 확인

```bash
# 환경 설정
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=20

# Topic 목록 확인
ros2 topic list

# Topic 데이터 모니터링
ros2 topic echo /int32_publisher

# Node 목록 확인
ros2 node list

# Node 정보 확인
ros2 node info /stm32_node
```

### 데이터 발행 테스트

```bash
# STM32로 Int32 데이터 전송
ros2 topic pub /int32_subscriber std_msgs/msg/Int32 "{data: 100}"

# cmd_vel 명령 전송 (전진)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# cmd_vel 명령 전송 (회전)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 실제 동작 확인 (STM32 콘솔):
# [BRIDGE] cmd_vel: L=1.00, A=0.50  (50번마다)
# [CONTROL] cmd_vel: L=0.93, R=1.08  (좌우 모터 속도)
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

### 2025.12 초
- Git 저장소 초기화
- GitHub 업로드 (ceceethpark/stm32_micro-ros)
- FreeRTOS 멀티태스킹 안정화 (Control/Sensor/MicroROS 독립 동작)
- printf를 SWO(PB3)로 변경하여 UART 충돌 해결 (U6.55)

### 2025.12.04
- UART 재설정: UART2(micro-ROS, 921600), UART3(printf, 115200), Timer3(Encoder)
- micro-ROS client key 20 설정
- WSL2 MicroXRCEAgent 설치 및 설정
- ROS2 Jazzy 설치 (Ubuntu 24.04)
- WSL2 네트워크 설정 (Mirrored Networking / Cyclone DDS Peer)

### 2025.12.05
- Hyper-V External Switch (Bridged Mode) 테스트 및 문서화
- vEthernet DHCP 제약 및 네트워크 위치 설정 이슈 해결
- Windows 네트워크 프로필 "개인" 설정으로 방화벽 문제 해결
- WSL2 네트워크 방법 우선순위 재정리 (Cyclone DDS Peer 가장 안정적)
- **WSL2 Ubuntu 22.04 + ROS2 Humble 환경 구축 성공**
  - Windows 10에서 Hyper-V Bridged Mode 사용하여 WSL을 192.168.0.65로 구성
  - ROS2 버전 호환성 해결: 192.168.0.101 (Humble) ↔ WSL 192.168.0.65 (Humble)
  - Multicast 기반 노드 디스커버리 정상 작동 확인
  - 중복 노드 문제 발견: 192.168.0.101이 eth0 + wlan0 동시 사용으로 인한 이슈

### 2025.12.06
- **양방향 ROS2-STM32 통신 완성** 🎉
  - **STM32 → ROS2**: IMU 데이터 JSON 송신 @ 5Hz via UART2 DMA TX
  - **ROS2 → STM32**: cmd_vel JSON 수신 @ 10Hz via UART2 DMA RX
- **DMA Circular Buffer 구현**
  - UART2 RX를 DMA Circular Buffer로 전환 (512 bytes)
  - 고속 안정적 수신 (921600 baud, Byte 모드)
  - Overrun 방지 및 실시간 JSON 파싱
- **Windows-WSL 브리지 아키텍처**
  - `serial_bridge.py` (Windows): COM5 ↔ UDP 8888/8889
  - `bridge_node.py` (WSL ROS2): ROS2 topics ↔ UDP
  - `udp_relay.py` (WSL): localhost:8889 → Windows IP:8889 (bridged mode 네트워크 격리 해결)
- **Differential Drive 계산 구현**
  - cmd_vel (linear, angular) → 좌우 모터 속도 자동 계산
  - Wheel base: 0.15m
  - Control Task에서 실시간 모터 제어
- **빌드 최적화**
  - GCC 플래그 호환성 문제 해결 (`-fcyclomatic-complexity` 제거)
  - 최종 빌드: 83252 text, 1064 data, 23432 bss (107748 bytes total)
- **안정성 검증**
  - 360초 이상 무중단 운영 확인
  - 메모리 누수 없음 (Heap 3536 일정 유지)
  - IMU 송신 1790+ 회, cmd_vel 수신/처리 정상

## 트러블슈팅

### WSL2 ROS2 통신 문제 해결 과정

#### 문제 1: WSL에서 ROS2 노드 발견 불가
- **증상**: `ros2 node list` 실행 시 빈 결과
- **원인**: WSL2 기본 NAT 네트워크(172.x.x.x)는 로컬 네트워크(192.168.0.x)와 multicast 통신 불가
- **해결**: Hyper-V Bridged Mode 적용하여 WSL을 192.168.0.65로 구성

#### 문제 2: ROS2 버전 불일치로 인한 노드 크래시
- **증상**: WSL(Jazzy)에서 메시지 발행 시 192.168.0.101(Humble) 노드 크래시 및 재부팅 필요
- **원인**: ROS2 Jazzy와 Humble 간 메시지 호환성 문제
- **해결**: WSL에 Ubuntu 22.04 + ROS2 Humble 재설치

#### 문제 3: 노드 디스커버리는 되지만 메시지 전달 안 됨
- **증상**: `ros2 topic list` 정상, 중복 노드 경고, WSL → 101 메시지 미전달
- **원인**: 192.168.0.101이 eth0(유선) + wlan0(무선) 동시 사용으로 중복 디스커버리
- **해결 방안**:
  1. WiFi 비활성화: `sudo rfkill block wifi`
  2. Cyclone DDS에서 eth0만 사용하도록 설정

#### 최종 성공 구성
```
Windows 10
└── WSL2 Ubuntu 22.04 (192.168.0.65) - Bridged Mode
    ├── ROS2 Humble
    ├── Cyclone DDS (multicast)
    └── Domain ID: 20
    
로컬 네트워크
├── 192.168.0.101 (Raspberry Pi, Humble) - 로봇
├── 192.168.0.154 (Humble) - 원격 제어
└── 192.168.0.65 (WSL2, Humble) - 개발 환경
```

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
