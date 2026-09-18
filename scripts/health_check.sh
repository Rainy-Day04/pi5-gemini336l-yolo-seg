#!/usr/bin/env bash
set -euo pipefail

required=(
  /camera/color/image_raw
  /camera/depth/image_raw
  /camera/color/camera_info
  /perception/gemini336l_yolo_seg/mask
  /perception/gemini336l_yolo_seg/detections_2d
  /perception/gemini336l_yolo_seg/objects_3d
)

topics="$(ros2 topic list)"
status=0
for topic in "${required[@]}"; do
  if grep -Fxq "${topic}" <<<"${topics}"; then
    echo "OK      ${topic}"
  else
    echo "MISSING ${topic}"
    status=1
  fi
done

echo
ros2 topic hz /camera/depth/image_raw --window 30 &
hz_pid=$!
sleep 4
kill "${hz_pid}" 2>/dev/null || true
wait "${hz_pid}" 2>/dev/null || true
exit "${status}"
