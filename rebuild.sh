#!/usr/bin/env bash
# Чистая пересборка workspace одним режимом (не смешивайте с --symlink-install).
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Готово. Выполните: source install/setup.bash"
