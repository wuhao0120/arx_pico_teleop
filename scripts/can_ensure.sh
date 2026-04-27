#!/bin/bash
################################################################################
# CAN 接口初始化辅助函数
#
# 被 start_rpc_server.sh / start_recording.sh source 使用。
# USB 重新插入后 slcand 可能变成僵尸进程，此脚本负责检测并重建 CAN 接口。
#
# 用法:
#   source scripts/can_ensure.sh
#   ensure_can_interface /dev/arxcan1 can1 "左臂"
################################################################################

# ensure_can_interface <usb_device> <can_interface> [label]
#   成功返回 0, 失败返回 1
ensure_can_interface() {
    local device="$1"
    local iface="$2"
    local label="${3:-$iface}"

    # 1) USB 设备是否存在
    if [ ! -e "$device" ]; then
        echo -e "\033[0;31m[ERR]\033[0m  $label: USB 设备 $device 不存在，请检查连线"
        return 1
    fi

    # 2) 接口已经 UP → 跳过
    if ip link show "$iface" 2>/dev/null | grep -q "state UP"; then
        echo -e "\033[0;32m[OK]\033[0m   $label: $iface 已就绪"
        return 0
    fi

    # 3) 接口不存在或 DOWN → 杀掉对应的 stale slcand，重建
    echo -e "\033[0;34m[INFO]\033[0m $label: 初始化 $iface ($device)..."

    # 仅杀掉绑定到该设备的 slcand 进程
    local stale_pids
    stale_pids=$(pgrep -f "slcand.*${device}" 2>/dev/null || true)
    if [ -n "$stale_pids" ]; then
        echo -e "\033[1;33m[WARN]\033[0m $label: 清理 stale slcand (PIDs: $stale_pids)"
        sudo kill -9 $stale_pids 2>/dev/null || true
        sleep 0.5
    fi

    # 删除可能残留的 DOWN 接口
    if ip link show "$iface" &>/dev/null; then
        sudo ip link set "$iface" down 2>/dev/null || true
        sudo ip link delete "$iface" 2>/dev/null || true
        sleep 0.3
    fi

    # 4) 启动 slcand
    sudo slcand -o -c -s8 "$device" "$iface"
    sleep 0.5
    sudo ip link set "$iface" up

    # 5) 验证
    if ip link show "$iface" 2>/dev/null | grep -q "state UP"; then
        echo -e "\033[0;32m[OK]\033[0m   $label: $iface 已启动"
        return 0
    else
        echo -e "\033[0;31m[ERR]\033[0m  $label: $iface 启动失败"
        return 1
    fi
}
