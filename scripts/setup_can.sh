#!/bin/bash
# 配置 CAN 接口用于升降电机控制
# 用法: sudo bash setup_can.sh [接口名] [波特率]
#   默认: can0 1000000 (1 Mbps)

INTERFACE=${1:-can0}
BITRATE=${2:-1000000}

if [ "$(id -u)" -ne 0 ]; then
    echo "需要 root 权限,请使用 sudo 运行" >&2
    exit 1
fi

echo "[setup_can] 配置 ${INTERFACE} @ ${BITRATE} bps, sample-point 0.875"

ip link set "${INTERFACE}" down 2>/dev/null
if ! ip link set "${INTERFACE}" type can bitrate "${BITRATE}" sample-point 0.875 2>/dev/null; then
    echo "[setup_can] 设置波特率失败,请确认接口 ${INTERFACE} 存在" >&2
    exit 1
fi

ip link set "${INTERFACE}" up

# 启用 bus-off 自动恢复(100ms)
ip link set "${INTERFACE}" type can restart-ms 100 2>/dev/null

sleep 0.5

STATE=$(ip -details link show "${INTERFACE}" 2>/dev/null | grep "can state" | awk '{print $3}')
BERR=$(ip -details link show "${INTERFACE}" 2>/dev/null | grep "berr-counter" | sed 's/.*berr-counter //')

if [ "${STATE}" = "ERROR-ACTIVE" ]; then
    echo "[setup_can] 成功: ${INTERFACE} 状态=${STATE} (${BERR})"
else
    echo "[setup_can] 警告: ${INTERFACE} 状态=${STATE} (${BERR})"
    echo "  若非 ERROR-ACTIVE,请检查: 1)电机是否上电 2)CAN接线 3)终端电阻 4)波特率是否匹配"
fi
