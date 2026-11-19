#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
# Graph tests publish project topics. Never inherit the live flight DDS domain.
export ROS_DOMAIN_ID="${FLAPPING_TEST_DOMAIN_ID:-196}"
export ROS_LOCALHOST_ONLY=1
if [[ ! $ROS_DOMAIN_ID =~ ^[0-9]+$ ]] || (( ROS_DOMAIN_ID < 1 || ROS_DOMAIN_ID > 232 )); then
  echo 'FLAPPING_TEST_DOMAIN_ID must be an integer in 1..232' >&2
  exit 1
fi
bash scripts/ros2.sh python3 /usr/bin/colcon test --base-paths src \
  --packages-select flapping_uav_msgs flapping_uav_gcs flapping_uav_bringup \
  --return-code-on-test-failure "$@"
bash scripts/ros2.sh python3 /usr/bin/colcon test-result --verbose
