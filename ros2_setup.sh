#!/bin/bash
# ROS2 Jazzy 환경 설정 스크립트

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ROS2 환경 설정
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Domain ID 설정
export ROS_DOMAIN_ID=20

# Cyclone DDS 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Cyclone DDS XML 설정 (있는 경우)
if [ -f "$HOME/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
fi

# 대화형 셸에서만 출력 (bashrc 자동 로딩 시)
if [[ $- == *i* ]]; then
    echo -e "${GREEN}=== ROS2 Jazzy 환경 설정 ===${NC}"
    echo -e "${YELLOW}✓${NC} ROS2 Jazzy sourced"
    echo -e "${YELLOW}✓${NC} ROS_DOMAIN_ID=20"
    echo -e "${YELLOW}✓${NC} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    
    if [ -f "$HOME/cyclonedds.xml" ]; then
        echo -e "${YELLOW}✓${NC} CYCLONEDDS_URI=file://$HOME/cyclonedds.xml"
    else
        echo -e "${YELLOW}⚠${NC} cyclonedds.xml not found (멀티캐스트 모드)"
    fi
    
    # 현재 네트워크 정보 표시
    echo ""
    echo -e "${GREEN}=== 네트워크 정보 ===${NC}"
    MY_IP=$(ip addr show eth0 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
    if [ -n "$MY_IP" ]; then
        echo -e "WSL IP: ${YELLOW}$MY_IP${NC}"
    fi
    
    echo ""
    echo -e "${GREEN}환경 설정 완료!${NC}"
    echo ""
fi
