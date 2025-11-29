# STM32CubeIDE CLI 빌드 스크립트

$IDE_PATH = "C:\ST\STM32CubeIDE_1.11.0\STM32CubeIDE\stm32cubeidec.exe"
$PROJECT_PATH = $PWD.Path
$PROJECT_NAME = "stm32_micro-ros"

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "STM32CubeIDE CLI 빌드" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "프로젝트: $PROJECT_NAME" -ForegroundColor White
Write-Host "경로: $PROJECT_PATH" -ForegroundColor Gray
Write-Host ""

# Clean
Write-Host "1. 프로젝트 Clean..." -ForegroundColor Yellow
& $IDE_PATH --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data $PROJECT_PATH -cleanBuild "$PROJECT_NAME/Debug"

if ($LASTEXITCODE -ne 0) {
    Write-Host "   ✗ Clean 실패!" -ForegroundColor Red
    exit $LASTEXITCODE
}

Write-Host "   ✓ Clean 완료" -ForegroundColor Green
Write-Host ""

# Build
Write-Host "2. 프로젝트 Build..." -ForegroundColor Yellow
& $IDE_PATH --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data $PROJECT_PATH -build "$PROJECT_NAME/Debug"

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "   ✗ 빌드 실패!" -ForegroundColor Red
    exit $LASTEXITCODE
}

Write-Host ""
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "✓ 빌드 완료!" -ForegroundColor Green
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""

# 결과 파일 확인
if (Test-Path "Debug\$PROJECT_NAME.elf") {
    $elfFile = Get-Item "Debug\$PROJECT_NAME.elf"
    Write-Host "생성된 파일:" -ForegroundColor Green
    Write-Host "  $($elfFile.FullName)" -ForegroundColor White
    Write-Host "  크기: $([math]::Round($elfFile.Length / 1KB, 2)) KB" -ForegroundColor Gray
}

Write-Host ""
