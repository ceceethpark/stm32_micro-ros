# STM32 micro-ROS 프로젝트 - CubeMX 사용 가이드

## CubeMX 코드 재생성 시 주의사항

이 프로젝트는 C++ 기반으로 작성되었으며, STM32CubeMX로 코드를 재생성할 때 다음 절차를 반드시 따라야 합니다.

---

## 📋 CubeMX 코드 재생성 절차

### 1단계: main.cpp → main.c 이름 변경
```powershell
# PowerShell에서
Rename-Item -Path "Core\Src\main.cpp" -NewName "main.c"
```

### 2단계: CubeMX에서 설정 변경 및 코드 생성
- STM32CubeMX에서 원하는 설정 변경 (UART, GPIO 등)
- **Project Manager → Project → Do not generate the main()** 체크 해제
- Generate Code 실행

### 3단계: main.c → main.cpp 이름 복원
```powershell
# PowerShell에서
Rename-Item -Path "Core\Src\main.c" -NewName "main.cpp"
```

### 4단계: 빌드 및 확인
```powershell
# 터미널 빌드 (가장 안정적)
powershell -ExecutionPolicy Bypass -File .\build_gcc.ps1
```

---

## ⚠️ 코드 재생성 후 자동 복구되는 항목

다음 코드들은 **USER CODE 블록** 안에 있어 CubeMX 재생성 후에도 자동으로 유지됩니다:

### ✅ 자동 보존되는 코드

#### 1. Task 선언 (`main.cpp` - USER CODE PV 블록)
```cpp
/* USER CODE BEGIN PV */
// Global task manager instance
task_class* pTaskManager = nullptr;

// Task handles and attributes (protected from CubeMX regeneration)
osThreadId_t microRosTaskHandle;
const osThreadAttr_t microRosTask_attributes = {
  .name = "MicroRosTask",
  .stack_size = 10240,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END PV */
```

#### 2. _write 함수 (UART 출력용)
```cpp
/* USER CODE BEGIN PFP */
#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len)
{
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
	if (HAL_UART_Transmit(&huart4, ptr, len, len) == HAL_OK)
		return len;
	else
		return 0;
}
/* USER CODE END PFP */
```

#### 3. Task Manager 초기화
```cpp
/* USER CODE BEGIN 2 */
// Task Manager 초기화
pTaskManager = new task_class(&huart1);
/* USER CODE END 2 */
```

#### 4. Queue 초기화
```cpp
/* USER CODE BEGIN RTOS_QUEUES */
// Initialize queues
if (pTaskManager != nullptr) {
  pTaskManager->initQueues();
}
/* USER CODE END RTOS_QUEUES */
```

#### 5. Task 생성
```cpp
/* USER CODE BEGIN RTOS_THREADS */
// All tasks created in USER CODE block (protected from CubeMX regeneration)

/* creation of MicroRosTask */
microRosTaskHandle = osThreadNew(task_class::microRosTaskWrapper, NULL, &microRosTask_attributes);

/* creation of ControlTask */
controlTaskHandle = osThreadNew(task_class::controlTaskWrapper, NULL, &controlTask_attributes);

/* creation of SensorTask */
sensorTaskHandle = osThreadNew(task_class::sensorTaskWrapper, NULL, &sensorTask_attributes);
/* USER CODE END RTOS_THREADS */
```

---

## 🏗️ 프로젝트 구조

```
stm32_micro-ros/
├── Core/
│   ├── Inc/
│   │   └── extern.h              # task_class 전역 선언
│   └── Src/
│       └── main.cpp              # 최소화된 main (초기화만)
├── class/
│   ├── task_class/               # ⭐ 모든 Task 로직
│   │   ├── task_class.h
│   │   └── task_class.cpp
│   ├── microros_class/           # micro-ROS 통신
│   ├── pwm_motor_class/          # 모터 제어
│   └── DataClass/                # 데이터 관리
├── Debug/
│   ├── makefile                  # task_class 포함 확인
│   ├── sources.mk                # class/task_class 경로 포함
│   ├── objects.list              # task_class.o 포함 확인
│   └── class/task_class/
│       └── subdir.mk
└── build_gcc.ps1                 # 터미널 빌드 스크립트
```

---

## 📊 Task 구조

### TASK 1: MicroRosTask (Priority: Normal, 10KB stack)
- **역할**: ROS 통신 전담
- **Subscribe**: cmd_vel, cam_pitch, cam_yaw
- **Publish**: IMU, Encoder, debug
- **주기**: 50ms (20Hz)

### TASK 2: ControlTask (Priority: High, 1KB stack)
- **역할**: 고속 실시간 모터 제어
- **주기**: 10ms (100Hz)
- **기능**: Differential drive, PWM 모터 제어

### TASK 3: SensorTask (Priority: Low, 1KB stack)
- **역할**: 센서 데이터 읽기/필터링
- **주기**: 50ms (20Hz)
- **센서**: IMU, Encoder

---

## 🔧 빌드 방법

### 터미널 빌드 (권장)
```powershell
powershell -ExecutionPolicy Bypass -File .\build_gcc.ps1
```

### IDE 빌드
1. **Clean Project** 실행
2. **Build Project** 실행
3. 증분 빌드는 불안정할 수 있으므로 항상 Clean 후 Build 권장

---

## ⚙️ UART 설정

- **UART1**: micro-ROS agent 통신 (115200 baud)
- **UART4**: 디버그 출력용 (_write 함수)

### micro-ROS agent 실행
```bash
# Windows
ros2 run micro_ros_agent micro_ros_agent serial --dev COM포트번호 -b 115200

# Linux
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

---

## 📝 빌드 결과 예시

```
=====================================
빌드 완료!
=====================================

   text    data     bss     dec     hex filename
  90700     696   57616  149012   24614 stm32_micro-ros.elf
```

---

## ❗ 문제 해결

### CubeMX 재생성 후 빌드 오류 발생 시

1. **Task 관련 선언이 사라졌는지 확인**
   - `main.cpp`에서 `USER CODE BEGIN PV` 블록 확인
   - Task handles 선언이 있는지 확인

2. **objects.list 확인**
   ```
   "./class/task_class/task_class.o"
   ```
   위 항목이 있는지 확인

3. **makefile 확인**
   ```makefile
   -include class/task_class/subdir.mk
   ```
   위 항목이 있는지 확인

4. **Clean 후 다시 빌드**
   ```powershell
   powershell -ExecutionPolicy Bypass -File .\build_gcc.ps1
   ```

---

## 📚 추가 정보

- **micro-ROS 버전**: Humble
- **STM32 MCU**: STM32F405RGTx (Cortex-M4, 168MHz)
- **RTOS**: FreeRTOS 10KB stack
- **C++ 표준**: GNU++14 (-fno-exceptions -fno-rtti)

---

## ✅ 체크리스트

CubeMX 재생성 후 다음 항목들을 확인하세요:

- [ ] main.c → main.cpp 이름 변경 완료
- [ ] Task 선언이 USER CODE PV 블록에 있음
- [ ] pTaskManager 초기화 코드가 USER CODE 2 블록에 있음
- [ ] Task 생성 코드가 USER CODE RTOS_THREADS 블록에 있음
- [ ] _write 함수가 USER CODE PFP 블록에 있음
- [ ] 터미널 빌드 성공
- [ ] 바이너리 크기 약 90KB (정상 범위: 85-95KB)

---

**작성일**: 2025-11-26  
**최종 빌드 크기**: 90,700 bytes (text) + 696 bytes (data) + 57,616 bytes (bss) = 149,012 bytes
