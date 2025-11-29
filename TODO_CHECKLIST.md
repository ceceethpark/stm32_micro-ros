# 🚀 micro-ROS 통합 체크리스트

## 현재 상태
- ✅ micro-ROS 유틸리티 다운로드 완료
- ✅ main.c에 micro-ROS 코드 추가 완료
- ✅ WSL2 Ubuntu 설치 확인
- ⬜ Makefile 생성 필요
- ⬜ micro-ROS 라이브러리 빌드 필요
- ⬜ Makefile 수정 필요
- ⬜ 프로젝트 빌드 필요

---

## 📋 다음 단계 (순서대로)

### ✅ STEP 1: STM32CubeMX 설정 및 Makefile 생성

**작업:**
1. `stm32_micro-ros.ioc` 파일을 STM32CubeMX로 열기
2. **Project Manager** 탭 클릭
3. **Project** 서브탭에서:
   - Toolchain/IDE: **Makefile** 선택
4. **(선택) UART DMA 설정:**
   - **Pinout & Configuration** 탭
   - **Connectivity** → **USART1**
   - **DMA Settings** 탭:
     - Add → USART1_RX: Mode=**Circular**, Priority=**Very High**
     - Add → USART1_TX: Priority=**Very High**
   - **NVIC Settings** 탭:
     - "USART1 global interrupt" 체크
5. **GENERATE CODE** 버튼 클릭

**확인:**
```powershell
Test-Path "c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros\Makefile"
```
→ `True`가 나와야 함

---

### ✅ STEP 2: micro-ROS 라이브러리 빌드

**방법 1: PowerShell 스크립트 사용 (권장)**
```powershell
cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
.\build_microros.ps1
```

**방법 2: 수동 실행**
```powershell
wsl -d Ubuntu
cd /mnt/c/Users/thpark/SynologyDrive/wspace/stm32_micro-ros
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
exit
```

**확인:**
```powershell
Test-Path "micro_ros_stm32cubemx_utils\microros_static_library\libmicroros\libmicroros.a"
```
→ `True`가 나와야 함

**참고:**
- 빌드 중 컴파일 플래그 확인 프롬프트가 나오면 `y` 입력
- 빌드 시간: 약 5-10분 소요

---

### ✅ STEP 3: Makefile 수정

**작업:**
생성된 `Makefile`을 열어서 **"build the application" 섹션 바로 앞에** 다음 내용 추가:

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

**찾는 방법:**
1. Makefile에서 `build the application` 또는 `# build the application` 검색
2. 해당 섹션 **바로 위에** 추가

---

### ✅ STEP 4: FreeRTOSConfig.h 힙 크기 확인 (선택)

**파일:** `Core\Inc\FreeRTOSConfig.h`

**확인할 내용:**
```c
#define configTOTAL_HEAP_SIZE ((size_t)(32 * 1024))  // 최소 32KB 권장
```

현재 값이 작으면 32KB 이상으로 증가

---

### ✅ STEP 5: 프로젝트 빌드

**명령:**
```powershell
cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
make -j8
```

**예상 출력:**
```
arm-none-eabi-gcc ...
...
arm-none-eabi-size build/stm32_micro-ros.elf
   text    data     bss     dec     hex filename
 123456    1234   12345  136035   21123 build/stm32_micro-ros.elf
arm-none-eabi-objcopy ...
```

**생성 파일:**
- `build/stm32_micro-ros.elf`
- `build/stm32_micro-ros.hex`
- `build/stm32_micro-ros.bin`

---

### ✅ STEP 6: 플래싱

**STM32CubeProgrammer 사용:**
1. STM32CubeProgrammer 실행
2. ST-Link 연결
3. `build/stm32_micro-ros.hex` 또는 `.bin` 파일 선택
4. Download 버튼 클릭

**또는 STM32CubeIDE에서:**
- Run → Debug 또는 Run

---

### ✅ STEP 7: micro-ROS Agent 실행 및 테스트

**터미널 1: Agent 실행**
```powershell
# ROS 2 환경 설정 (경로는 설치 위치에 따라 다름)
C:\ros2_humble\local_setup.bat

# COM 포트 확인 (장치 관리자)
# Agent 실행
ros2 run micro_ros_agent micro_ros_agent serial --dev COM3 -b 115200
```

**예상 출력:**
```
[1234567890.123456] info     | TermiosAgentLinux.cpp | init | running...
[1234567890.234567] info     | Root.cpp | create_client | create
[1234567890.345678] info     | SessionManager.hpp | establish_session | session established
```

**터미널 2: 토픽 확인**
```powershell
ros2 topic list
# 출력: /stm32_counter

ros2 topic echo /stm32_counter
# 출력:
# data: 0
# ---
# data: 1
# ---
# data: 2
# ...
```

---

## 🔧 트러블슈팅

### 문제: Makefile이 생성되지 않음
- **해결:** STM32CubeMX에서 Toolchain/IDE가 **Makefile**로 설정되었는지 재확인

### 문제: Docker 빌드 실패
- **해결:** WSL2에서 Docker Desktop 설치 필요
  ```powershell
  wsl --install
  # Docker Desktop for Windows 설치
  ```

### 문제: make 명령 없음
- **해결:** ARM GCC 툴체인 및 Make 설치 필요
  - STM32CubeIDE 설치 시 포함됨
  - 또는 별도로 [GNU ARM Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) 설치

### 문제: Agent 연결 실패
- **해결:**
  1. COM 포트 번호 확인 (장치 관리자)
  2. UART 보레이트 확인 (115200)
  3. STM32 재부팅

### 문제: 메모리 부족
- **해결:**
  1. `FreeRTOSConfig.h`에서 `configTOTAL_HEAP_SIZE` 증가
  2. Task 스택 크기 확인 (현재: 10240)

---

## 📚 참고 문서

- **QUICKSTART.md**: 빠른 시작 가이드
- **MICROROS_SETUP_GUIDE.md**: 상세 설정 가이드
- **main_microros_example.c**: 추가 예제 코드
- **micro_ros_stm32cubemx_utils/README.md**: 공식 문서

---

## 🎯 현재 해야 할 일

**지금 바로:**
1. ✅ STM32CubeMX 열기 → Makefile 생성 (STEP 1)
2. ✅ `build_microros.ps1` 실행 (STEP 2)
3. ✅ Makefile 수정 (STEP 3)
4. ✅ `make -j8` 빌드 (STEP 5)

**순서대로 진행하세요!** ✨
