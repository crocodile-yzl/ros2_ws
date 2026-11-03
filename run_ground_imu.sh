#!/usr/bin/env bash
# 常驻运行完整栈（深度 + IMU + 分割 + RViz），Ctrl-C 退出。
#
# 与 verify_ground_imu.sh 的区别：这个不采样、不杀进程，只在**前台**跑 launch，
# 关掉终端或按 Ctrl-C 才结束，适合长时间观察 RViz。
#
# 用法：
#   ./run_ground_imu.sh            # 带 RViz（默认）
#   ./run_ground_imu.sh --no-rviz  # 只看话题
set -o pipefail

# 会话 /dev 被 tmpfs 覆盖时，先在私有挂载命名空间揭开再重入自己
if [ -z "${GROUND_IMU_REVEALED:-}" ] && [ ! -d /dev/bus/usb ]; then
    export GROUND_IMU_REVEALED=1
    exec unshare -m bash -c 'umount -l /dev 2>/dev/null || true; exec bash "$0" "$@"' "$0" "$@"
fi

cd /home/y/ros2_ws
source /opt/ros/humble/setup.bash
source install_y/setup.bash

RVIZ=true
[ "${1:-}" = "--no-rviz" ] && RVIZ=false

exec ros2 launch d435i_ransac_imu d435i_ransac_imu.launch.py rviz:="$RVIZ"
