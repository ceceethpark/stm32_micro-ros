# ROS2 방화벽 규칙 설정
# 관리자 권한으로 실행 필요

Write-Host "ROS2 DDS 방화벽 규칙 추가 중..." -ForegroundColor Green

# 1. DDS Discovery 포트 (UDP 7400-7500)
New-NetFirewallRule -DisplayName "ROS2 DDS Discovery" -Direction Inbound -Protocol UDP -LocalPort 7400-7500 -Action Allow -Profile Any -ErrorAction SilentlyContinue
Write-Host "✓ ROS2 DDS Discovery (UDP 7400-7500) 규칙 추가됨" -ForegroundColor Cyan

# 2. DDS Data 포트 (UDP 7410-7610)
New-NetFirewallRule -DisplayName "ROS2 DDS Data UDP" -Direction Inbound -Protocol UDP -LocalPort 7410-7610 -Action Allow -Profile Any -ErrorAction SilentlyContinue
Write-Host "✓ ROS2 DDS Data UDP (7410-7610) 규칙 추가됨" -ForegroundColor Cyan

# 3. DDS Data 포트 (TCP 7410-7610)
New-NetFirewallRule -DisplayName "ROS2 DDS Data TCP" -Direction Inbound -Protocol TCP -LocalPort 7410-7610 -Action Allow -Profile Any -ErrorAction SilentlyContinue
Write-Host "✓ ROS2 DDS Data TCP (7410-7610) 규칙 추가됨" -ForegroundColor Cyan

# 4. Outbound 규칙도 추가
New-NetFirewallRule -DisplayName "ROS2 DDS Discovery Out" -Direction Outbound -Protocol UDP -LocalPort 7400-7500 -Action Allow -Profile Any -ErrorAction SilentlyContinue
Write-Host "✓ ROS2 DDS Discovery Outbound 규칙 추가됨" -ForegroundColor Cyan

Write-Host "`n방화벽 규칙 설정 완료!" -ForegroundColor Green
Write-Host "WSL에서 'ros2 node list'로 테스트하세요." -ForegroundColor Yellow

# vEthernet 네트워크 프로필 확인
Write-Host "`n=== 네트워크 프로필 확인 ===" -ForegroundColor Green
Get-NetConnectionProfile | Where-Object {$_.InterfaceAlias -like "*vEthernet*" -or $_.InterfaceAlias -like "*WSL*"} | Format-Table InterfaceAlias, NetworkCategory, IPv4Connectivity

Write-Host "`n⚠ vEthernet 네트워크가 'Public'이면 'Private'으로 변경하세요:" -ForegroundColor Yellow
Write-Host "Set-NetConnectionProfile -InterfaceAlias '인터페이스이름' -NetworkCategory Private" -ForegroundColor Cyan
