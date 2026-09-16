#!/usr/bin/env bash
# Прописывает CYCLONEDDS_URI в ~/.bashrc (лимит участников DDS для большого стека).
set -e
WS="$(cd "$(dirname "$0")" && pwd)"
CFG="$WS/src/robot_navigation/config/cyclonedds.xml"
LINE="export CYCLONEDDS_URI=file://$CFG"
grep -qF "CYCLONEDDS_URI" ~/.bashrc && sed -i '/CYCLONEDDS_URI/d' ~/.bashrc
echo "$LINE" >> ~/.bashrc
grep -qF "RMW_IMPLEMENTATION" ~/.bashrc || echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "Добавлено в ~/.bashrc:"
echo "  $LINE"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "Выполните: source ~/.bashrc"
