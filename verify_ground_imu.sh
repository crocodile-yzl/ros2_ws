#!/usr/bin/env bash
# 验证「一个进程同时发深度+IMU」以及 IMU 重力先验是否生效。
#
# 设计原则：只以**真实启动结果**作为结论。v4l2-ctl 的探测只作参考打印，
# 不再据它判断“被占用”并提前退出——那个代理指标曾经误报过。
#
# 背景：
#   1) 会话 /dev 被 tmpfs 覆盖时，真实 devtmpfs 里的 /dev/bus/usb、/dev/video*
#      看不到，先在私有挂载命名空间里 umount 掉这层覆盖。
#   2) librealsense 独占整个 USB 设备，同一台相机只能被一个进程打开。
#
# 用法： ./verify_ground_imu.sh [--rviz]
# 注意：ROS 的 setup.bash 会引用未定义变量，所以这里不用 set -u。
set -o pipefail

# --- 1. 揭开 /dev 覆盖层后重新进入自己 ---
if [ -z "${VERIFY_DEV_REVEALED:-}" ]; then
    if [ ! -d /dev/bus/usb ]; then
        export VERIFY_DEV_REVEALED=1
        exec unshare -m bash -c 'umount -l /dev 2>/dev/null || true; exec bash "$0" "$@"' "$0" "$@"
    fi
fi

cd /home/y/ros2_ws
source /opt/ros/humble/setup.bash
source install_y/setup.bash

RVIZ=false
[ "${1:-}" = "--rviz" ] && RVIZ=true

# --- 2. 找深度节点并做一次参考性探测（不影响是否继续）---
DEPTH_DEV=""
if command -v v4l2-ctl >/dev/null 2>&1; then
    for d in /dev/video*; do
        [ -e "$d" ] || continue
        if v4l2-ctl -d "$d" --get-fmt-video 2>/dev/null | grep -q "'Z16 '"; then
            DEPTH_DEV="$d"
            break
        fi
    done
fi

if [ -n "$DEPTH_DEV" ]; then
    echo "深度节点: $DEPTH_DEV"
    if timeout 5 v4l2-ctl -d "$DEPTH_DEV" --stream-mmap --stream-count=1 >/dev/null 2>&1; then
        echo "预检: 探测取帧成功（仅供参考）"
    else
        echo "预检: 探测取帧失败（仅供参考，可能被占用，也可能只是探测本身的限制）"
    fi
else
    echo "预检: 没找到 Z16 节点（未安装 v4l2-ctl 或相机未就绪），继续尝试启动"
fi

# --- 3. 起 launch（独立进程组，退出时只清理自己）---
LOG=/home/y/ros2_ws/verify_ground_imu.log
setsid ros2 launch d435i_ransac_imu d435i_ransac_imu.launch.py rviz:="$RVIZ" > "$LOG" 2>&1 &
LP=$!
# 只杀自己这个进程组，避免误伤机器上另一套 d435i 实例
cleanup() { kill -- -"$LP" 2>/dev/null; }
trap cleanup EXIT INT TERM
sleep 12

echo
echo "=== 节点日志 ==="
grep -E "D435i depth|IMU:|imu_prior|IMU gravity|plane fit|ground: n|error|died" "$LOG" | head -12

# --- 4. 以真实启动结果判定 ---
if grep -q "Device or resource busy" "$LOG"; then
    echo
    echo "[失败] d435i_cloud_node 拿不到相机：Device or resource busy。" >&2
    echo "       说明此刻确实有别的进程开着深度节点，可在宿主机上定位：" >&2
    echo "         sudo fuser -v /dev/video*" >&2
    echo "         sudo lsof /dev/video* 2>/dev/null | grep -w video" >&2
    echo "         docker ps | grep -i ros    # 若是另一个容器在跑" >&2
    echo "       也可能只是上一次运行还没释放，等 2 秒再试。" >&2
    exit 2
fi

if grep -q "process has died" "$LOG"; then
    echo
    echo "[失败] 有节点启动失败，完整日志：$LOG" >&2
    exit 2
fi

echo
echo "=== 频率 ==="
for t in /d435i/points /d435i/accel/sample /d435i/gyro/sample; do
    printf '%-24s' "$t"
    # 先整段收下来再匹配：直接 grep -m1 会因为提前关闭管道让 ros2 收到 SIGPIPE，
    # 返回非零被误判成“无数据”。
    out=$(timeout 6 ros2 topic hz "$t" 2>&1)
    rate=$(printf '%s\n' "$out" | grep -m1 "average rate")
    if [ -n "$rate" ]; then echo "$rate"; else echo "无数据"; fi
done

echo
echo "=== /ground/plane ==="
out=$(timeout 6 ros2 topic echo /ground/plane --once 2>&1)
printf '%s\n' "$out" | head -6

echo
echo "完整日志: $LOG"
