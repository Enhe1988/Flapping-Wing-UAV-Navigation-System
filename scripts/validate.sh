#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
bash scripts/build.sh
bash scripts/test.sh
bash scripts/ros2.sh python3 scripts/check_uwb_demo.py
bash scripts/ros2.sh python3 scripts/check_uwb_serial.py
