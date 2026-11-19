#!/usr/bin/env bash
set -e
flight_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$flight_workspace/scripts/ros2.sh" ros2 run flapping_uav_gcs flight_log.py "$@"
