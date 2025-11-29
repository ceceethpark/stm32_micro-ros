# STM32CubeIDE에서 micro-ROS 설정 가이드

## 개요
STM32CubeIDE는 Eclipse 기반 IDE로 Makefile 대신 자체 빌드 시스템을 사용합니다.
이 가이드는 STM32CubeIDE에서 micro-ROS를 통합하는 방법을 설명합니다.

---

## 1단계: micro-ROS 라이브러리 빌드

### PowerShell 스크립트 사용 (권장)

```powershell
cd c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros
.\build_microros_cubeide.ps1
```

### 또는 수동 빌드 (WSL2)

```bash
wsl -d Ubuntu
cd /mnt/c/Users/thpark/SynologyDrive/wspace/stm32_micro-ros
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble
exit
```

**빌드 결과:**
- 라이브러리: `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a`
- 헤더: `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/microros_include/`

---

## 2단계: STM32CubeIDE 프로젝트 설정

### A. Include 경로 추가

1. **프로젝트 우클릭** → **Properties**
2. **C/C++ Build** → **Settings**
3. **Tool Settings** 탭 클릭
4. **MCU GCC Compiler** → **Include paths** 선택
5. **Add...** (초록색 + 아이콘) 클릭
6. **Workspace...** 버튼 클릭하여 다음 경로 선택:
   ```
   micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/microros_include
   ```
   또는 직접 입력:
   ```
   "${workspace_loc:/${ProjName}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/microros_include}"
   ```

### B. 라이브러리 추가

같은 **Settings** 창에서:

1. **MCU GCC Linker** → **Libraries** 선택

2. **Libraries (-l)** 섹션:
   - **Add...** 클릭
   - 입력: `microros` (lib 접두사와 .a 확장자 제외)

3. **Library search path (-L)** 섹션:
   - **Add...** 클릭
   - **Workspace...** 버튼으로 경로 선택:
     ```
     micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
     ```
   - 또는 직접 입력:
     ```
     "${workspace_loc:/${ProjName}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros}"
     ```

4. **Apply and Close** 클릭

---

## 3단계: micro-ROS 소스 파일 추가

### 방법 A: 드래그 앤 드롭

1. Windows 탐색기에서 다음 폴더 열기:
   ```
   c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros\micro_ros_stm32cubemx_utils\extra_sources
   ```

2. 다음 파일들을 STM32CubeIDE의 **Core/Src** 폴더로 드래그:
   - `microros_time.c`
   - `microros_allocators.c`
   - `custom_memory_manager.c`

3. `microros_transports` 폴더에서 전송 방식에 맞는 파일 추가:
   - **DMA 사용**: `microros_transports/dma_transport.c`
   - **Interrupt 사용**: `microros_transports/it_transport.c`
   - **USB CDC**: `microros_transports/usb_cdc_transport.c`

4. 대화상자에서 **"Copy files"** 선택

### 방법 B: Import 사용

1. **Core/Src** 폴더 우클릭 → **Import...**
2. **General** → **File System** → **Next**
3. **From directory:** Browse 클릭
4. 경로 선택:
   ```
   c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros\micro_ros_stm32cubemx_utils\extra_sources
   ```
5. 필요한 파일 체크:
   - `microros_time.c`
   - `microros_allocators.c`
   - `custom_memory_manager.c`
   - `microros_transports/dma_transport.c` (전송 방식에 따라)
6. **Finish** 클릭

---

## 4단계: FreeRTOS 힙 크기 조정

### FreeRTOSConfig.h 수정

**파일:** `Core/Inc/FreeRTOSConfig.h`

다음 값을 찾아서 수정:

```c
// 기존
#define configTOTAL_HEAP_SIZE ((size_t)(15 * 1024))

// 변경 (최소 32KB 권장)
#define configTOTAL_HEAP_SIZE ((size_t)(32 * 1024))
```

---

## 5단계: 프로젝트 빌드

1. **Project** → **Clean...** → 프로젝트 선택 → **Clean**
2. **Project** → **Build All** (Ctrl+B)

### 예상 빌드 출력:

```
Building target: stm32_micro-ros.elf
Finished building target: stm32_micro-ros.elf
 
arm-none-eabi-size  stm32_micro-ros.elf
   text    data     bss     dec     hex filename
 123456    2345   23456  149257   24739 stm32_micro-ros.elf
Finished building: default.size.stdout
 
arm-none-eabi-objdump -h -S stm32_micro-ros.elf > "stm32_micro-ros.list"
```

---

## 6단계: 플래싱 및 디버깅

### 플래싱

1. **Run** → **Run Configurations...**
2. **STM32 Cortex-M C/C++ Application** 더블클릭
3. **C/C++ Application:** `Debug/stm32_micro-ros.elf` 확인
4. **Run** 버튼 클릭

### 디버깅

1. **Run** → **Debug** (F11)
2. Perspective 전환 확인 메시지에서 **Switch** 클릭

---

## 7단계: micro-ROS Agent 연결 및 테스트

### PC에서 Agent 실행 (Windows PowerShell)

```powershell
# ROS 2 환경 설정
C:\ros2_humble\local_setup.bat

# COM 포트 확인 (장치 관리자)
# STM32 연결 후 나타나는 COM 포트 번호 확인

# Agent 실행
ros2 run micro_ros_agent micro_ros_agent serial --dev COM3 -b 115200
```

### 토픽 확인 (새 터미널)

```powershell
ros2 topic list
# 출력: /stm32_counter

ros2 topic echo /stm32_counter
# 출력:
# data: 0
# ---
# data: 1
# ---
```

---

## 트러블슈팅

### 1. 빌드 오류: "undefined reference to `rcl_...`"

**원인:** 라이브러리 링크 설정 문제

**해결:**
- Properties → C/C++ Build → Settings → MCU GCC Linker → Libraries 재확인
- Library search path가 올바른지 확인
- Clean 후 Rebuild

### 2. 빌드 오류: "rcl/rcl.h: No such file or directory"

**원인:** Include 경로 설정 문제

**해결:**
- Properties → C/C++ Build → Settings → MCU GCC Compiler → Include paths 재확인
- 경로가 정확한지 확인

### 3. 런타임 오류: HardFault 또는 메모리 부족

**원인:** 힙/스택 부족

**해결:**
1. `FreeRTOSConfig.h`에서 `configTOTAL_HEAP_SIZE` 증가 (최소 32KB)
2. `main.c`에서 Task 스택 크기 확인 (현재: 10240)
3. `.ld` 파일에서 힙/스택 섹션 크기 확인

### 4. Agent 연결 실패

**원인:** UART 설정 또는 COM 포트 문제

**해결:**
1. 장치 관리자에서 COM 포트 번호 재확인
2. STM32CubeMX에서 UART DMA 설정 확인
3. 보레이트 일치 확인 (115200)
4. STM32 재부팅

---

## 추가 설정 (선택)

### Pre-build 자동화 (Docker 설치 시)

**Properties** → **C/C++ Build** → **Settings** → **Build Steps** 탭

**Pre-build steps** → **Command:**
```bash
docker pull microros/micro_ros_static_library_builder:humble && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble
```

> ⚠️ **주의:** 이 설정은 매 빌드마다 라이브러리를 재빌드하므로 시간이 오래 걸립니다.

---

## 요약 체크리스트

- [ ] micro-ROS 라이브러리 빌드 (`build_microros_cubeide.ps1` 실행)
- [ ] Include 경로 추가 (Properties → Compiler → Include paths)
- [ ] 라이브러리 추가 (Properties → Linker → Libraries)
- [ ] 소스 파일 추가 (extra_sources/*.c)
- [ ] FreeRTOS 힙 크기 증가 (32KB)
- [ ] 프로젝트 Clean & Build
- [ ] 플래싱
- [ ] Agent 실행 및 테스트

---

## 참고 자료

- **TODO_CHECKLIST.md**: 전체 작업 흐름
- **QUICKSTART.md**: 빠른 참조
- **MICROROS_SETUP_GUIDE.md**: Makefile 기반 설정
- **main_microros_example.c**: 코드 예제
- [micro-ROS 공식 문서](https://micro.ros.org/)
