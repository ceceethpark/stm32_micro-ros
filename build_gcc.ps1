# ARM GCC 직접 사용 빌드 스크립트

# ARM GCC 툴체인 경로 설정
$GCC_PATH = Get-ChildItem "C:\ST\STM32CubeIDE_1.11.0" -Filter "arm-none-eabi-gcc.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Directory
$env:PATH = "$GCC_PATH;$env:PATH"

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "ARM GCC 빌드" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""

# GCC 버전 확인
Write-Host "GCC 버전:" -ForegroundColor Yellow
arm-none-eabi-gcc --version | Select-Object -First 1
Write-Host ""

# 빌드 디렉토리로 이동
if (!(Test-Path "Debug")) {
    Write-Host "Debug 폴더가 없습니다. 먼저 STM32CubeIDE에서 한 번 빌드하세요." -ForegroundColor Red
    exit 1
}

Write-Host "빌드 시작..." -ForegroundColor Yellow
Write-Host ""

# make 실행
cd Debug
make -j8 all

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "=====================================" -ForegroundColor Cyan
    Write-Host "✓ 빌드 완료!" -ForegroundColor Green
    Write-Host "=====================================" -ForegroundColor Cyan
    
    # 결과 출력
    if (Test-Path "stm32_micro-ros.elf") {
        Write-Host ""
        arm-none-eabi-size stm32_micro-ros.elf
    }
} else {
    Write-Host ""
    Write-Host "✗ 빌드 실패!" -ForegroundColor Red
}

cd ..
