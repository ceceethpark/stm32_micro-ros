@echo off
REM micro-ROS 라이브러리 빌드 스크립트 (WSL2 Ubuntu 사용)

echo ====================================
echo micro-ROS 라이브러리 빌드 시작
echo ====================================
echo.

REM 현재 디렉토리의 Windows 경로를 WSL 경로로 변환
set CURRENT_DIR=%cd%
set WSL_PATH=/mnt/c/Users/thpark/SynologyDrive/wspace/stm32_micro-ros

echo WSL2 Ubuntu 시작 중...
wsl -d Ubuntu bash -c "cd %WSL_PATH% && docker pull microros/micro_ros_static_library_builder:humble"

echo.
echo Docker 이미지 다운로드 완료!
echo.

echo micro-ROS 라이브러리 빌드 중...
wsl -d Ubuntu bash -c "cd %WSL_PATH% && docker run --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble"

echo.
echo ====================================
echo 빌드 완료!
echo ====================================
echo.
echo 생성된 라이브러리:
echo micro_ros_stm32cubemx_utils\microros_static_library\libmicroros\libmicroros.a
echo.
pause
