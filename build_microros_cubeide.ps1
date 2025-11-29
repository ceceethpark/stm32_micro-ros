# STM32CubeIDE용 micro-ROS 라이브러리 빌드 스크립트

Write-Host "====================================" -ForegroundColor Cyan
Write-Host "STM32CubeIDE용 micro-ROS 빌드" -ForegroundColor Cyan
Write-Host "====================================" -ForegroundColor Cyan
Write-Host ""

$PROJECT_PATH = "c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros"
$WSL_PATH = "/mnt/c/Users/thpark/SynologyDrive/wspace/stm32_micro-ros"

Write-Host "WSL2 Ubuntu에서 빌드 시작..." -ForegroundColor Yellow
Write-Host ""

# Docker 이미지 다운로드
Write-Host "1. Docker 이미지 다운로드 중..." -ForegroundColor Cyan
wsl -d Ubuntu bash -c "cd $WSL_PATH && docker pull microros/micro_ros_static_library_builder:humble"

if ($LASTEXITCODE -ne 0) {
    Write-Host "   ✗ Docker 이미지 다운로드 실패!" -ForegroundColor Red
    exit
}

Write-Host "   ✓ 완료!" -ForegroundColor Green
Write-Host ""

# STM32CubeIDE용 라이브러리 빌드
Write-Host "2. micro-ROS 라이브러리 빌드 중 (STM32CubeIDE용)..." -ForegroundColor Yellow
Write-Host "   (이 작업은 5-10분 소요될 수 있습니다)" -ForegroundColor Gray
Write-Host ""

wsl -d Ubuntu bash -c "cd $WSL_PATH && docker run --rm -v `$(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble"

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "   ✗ 빌드 실패!" -ForegroundColor Red
    exit
}

Write-Host ""
Write-Host "====================================" -ForegroundColor Cyan
Write-Host "✓ 빌드 완료!" -ForegroundColor Green
Write-Host "====================================" -ForegroundColor Cyan
Write-Host ""

# 라이브러리 파일 확인
$LIB_PATH = "$PROJECT_PATH\micro_ros_stm32cubemx_utils\microros_static_library_ide\libmicroros"
if (Test-Path "$LIB_PATH\libmicroros.a") {
    Write-Host "생성된 파일:" -ForegroundColor Green
    Write-Host "  라이브러리: $LIB_PATH\libmicroros.a" -ForegroundColor White
    Write-Host "  헤더: $LIB_PATH\microros_include\" -ForegroundColor White
    Write-Host ""
    $fileInfo = Get-Item "$LIB_PATH\libmicroros.a"
    Write-Host "  크기: $([math]::Round($fileInfo.Length / 1MB, 2)) MB" -ForegroundColor Gray
} else {
    Write-Host "⚠ 라이브러리 파일을 찾을 수 없습니다." -ForegroundColor Yellow
}

Write-Host ""
Write-Host "다음 단계:" -ForegroundColor Cyan
Write-Host "  1. STM32CubeIDE에서 프로젝트 Properties 열기" -ForegroundColor White
Write-Host "  2. Include 경로 및 라이브러리 추가 (CUBEIDE_SETUP.md 참조)" -ForegroundColor White
Write-Host "  3. 소스 파일 추가" -ForegroundColor White
Write-Host "  4. 프로젝트 빌드" -ForegroundColor White
Write-Host ""
