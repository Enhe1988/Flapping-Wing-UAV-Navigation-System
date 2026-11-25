#!/usr/bin/env bash
# Isolate this ROS2 workspace from the user's ROS1/other workspace environment.
set -e
if [[ $# -eq 0 || ${1:-} == --help || ${1:-} == -h ]]; then
  cat <<'USAGE'
用法：bash scripts/ros2.sh <命令> [参数...]
本脚本只为所执行的命令加载 ROS2，不会修改当前终端环境。

启动通信（默认开启实际发送）：
  bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py

启动备用通信（10.0.0.100:14550，关闭遥控转发）：
  bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication_backup.launch.py

仅接收调试（关闭所有UDP发送）：
  bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py tx_enabled:=false

进入已加载 ROS2 的子终端：
  bash scripts/ros2.sh bash --noprofile --norc -i
USAGE
  if [[ $# -eq 0 ]]; then exit 2; fi
  exit 0
fi
flapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${FLAPPING_CLEAN_SHELL:-0} != 1 ]]; then
  exec env -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    -u ROS_ROOT -u ROS_MASTER_URI -u ROS_ETC_DIR -u ROS_PACKAGE_PATH \
    -u ROSLISP_PACKAGE_DIRECTORIES -u PYTHONPATH -u LD_LIBRARY_PATH \
    -u CMAKE_PREFIX_PATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PKG_CONFIG_PATH -u VIRTUAL_ENV \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    FLAPPING_CLEAN_SHELL=1 bash --noprofile --norc "$0" "$@"
fi
cd "$flapping_root"
flapping_distro="${FLAPPING_ROS_DISTRO:-foxy}"
source /opt/ros/"$flapping_distro"/setup.bash
if [[ ${ROS_VERSION:-} != 2 ]]; then echo 'A ROS2 underlay is required' >&2; exit 1; fi
if [[ -f install/local_setup.bash ]]; then source install/local_setup.bash; fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-$flapping_root/log/ros}"
mkdir -p "$ROS_LOG_DIR"
exec "$@"
