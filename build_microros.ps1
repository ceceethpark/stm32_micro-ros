# micro-ROS 라이브러리 빌드 스크립트 (PowerShell)

Write-Host "====================================" -ForegroundColor Cyan
Write-Host "micro-ROS 라이브러리 빌드 시작" -ForegroundColor Cyan
Write-Host "====================================" -ForegroundColor Cyan
Write-Host ""

$PROJECT_PATH = "c:\Users\thpark\SynologyDrive\wspace\stm32_micro-ros"
$WSL_PATH = "/mnt/c/Users/thpark/SynologyDrive/wspace/stm32_micro-ros"

Write-Host "1. Makefile 확인 중..." -ForegroundColor Yellow
if (Test-Path "$PROJECT_PATH\Makefile") {
    Write-Host "   ✓ Makefile 발견!" -ForegroundColor Green
} else {
    Write-Host "   ✗ Makefile이 없습니다!" -ForegroundColor Red
    Write-Host "   → STM32CubeMX에서 먼저 Makefile을 생성하세요." -ForegroundColor Red
    Write-Host "   → Project Manager → Toolchain/IDE: Makefile 선택" -ForegroundColor Yellow
    Write-Host ""
    Read-Host "계속하려면 Enter를 누르세요"
    exit
}

Write-Host ""
Write-Host "2. WSL2 Ubuntu 시작 중..." -ForegroundColor Yellow

# Docker 이미지 다운로드
Write-Host "   Docker 이미지 다운로드 중..." -ForegroundColor Cyan
wsl -d Ubuntu bash -c "cd $WSL_PATH && docker pull microros/micro_ros_static_library_builder:humble"

if ($LASTEXITCODE -ne 0) {
    Write-Host "   ✗ Docker 이미지 다운로드 실패!" -ForegroundColor Red
    Write-Host "   → WSL2에서 Docker가 실행 중인지 확인하세요." -ForegroundColor Yellow
    Write-Host "   → 'wsl -d Ubuntu' 실행 후 'docker ps'로 확인" -ForegroundColor Yellow
    exit
}

Write-Host "   ✓ Docker 이미지 다운로드 완료!" -ForegroundColor Green
Write-Host ""

# micro-ROS 라이브러리 빌드
Write-Host "3. micro-ROS 라이브러리 빌드 중..." -ForegroundColor Yellow
Write-Host "   (컴파일 플래그 확인 프롬프트가 나오면 'y' 입력)" -ForegroundColor Cyan
Write-Host ""

wsl -d Ubuntu bash -c "cd $WSL_PATH && docker run --rm -v `$(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble"

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
$LIB_PATH = "$PROJECT_PATH\micro_ros_stm32cubemx_utils\microros_static_library\libmicroros\libmicroros.a"
if (Test-Path $LIB_PATH) {
    Write-Host "생성된 라이브러리:" -ForegroundColor Green
    Write-Host "  $LIB_PATH" -ForegroundColor White
    Write-Host ""
    $fileInfo = Get-Item $LIB_PATH
    Write-Host "  크기: $([math]::Round($fileInfo.Length / 1MB, 2)) MB" -ForegroundColor Gray
} else {
    Write-Host "⚠ 라이브러리 파일을 찾을 수 없습니다." -ForegroundColor Yellow
}

Write-Host ""
Write-Host "다음 단계:" -ForegroundColor Cyan
Write-Host "  1. Makefile에 micro-ROS 설정 추가 (QUICKSTART.md 참조)" -ForegroundColor White
Write-Host "  2. 'make -j8' 명령으로 프로젝트 빌드" -ForegroundColor White
Write-Host ""
