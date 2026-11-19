#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
flapping_cmake_args=(-DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_STANDARD=17
  -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
if [[ ${FLAPPING_ROS_DISTRO:-foxy} == foxy ]]; then
  flapping_cmake_args+=("-DCMAKE_PROJECT_INCLUDE_BEFORE=$PWD/cmake/foxy_rosidl_compat.cmake")
fi
bash scripts/ros2.sh python3 /usr/bin/colcon build --base-paths src \
  --symlink-install --parallel-workers 2 \
  --packages-up-to flapping_uav_gcs flapping_uav_bringup \
  --cmake-args "${flapping_cmake_args[@]}" "$@"
