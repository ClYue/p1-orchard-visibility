#!/bin/bash
# M3 验证脚本：无头启动果园世界 -> 等 UGV 话题 -> 读位姿 -> cmd_vel 驱动 -> 再读位姿
set -u
ROOT="/mnt/c/Users/yue_c/OneDrive - hnuahe.edu.cn/科研/论文/地空协同青基/08_Gazebo仿真/p1_gz_sim"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/models"

ign gazebo -s -r "$ROOT/worlds/orchard.world" > /tmp/orchard_run.log 2>&1 &
SPID=$!

UP=0
for i in $(seq 1 24); do
  sleep 5
  U=$(timeout 8 ign topic -l 2>/dev/null | grep -c "ugv")
  echo "t=$((i*5))s ugv_topics=$U"
  if [ "${U:-0}" -gt 0 ]; then UP=1; break; fi
done

if [ "$UP" -ne 1 ]; then
  echo "RESULT: server topics never appeared"
  echo "=== log errors ==="
  grep -iE "error|failed" /tmp/orchard_run.log | head -8
  echo "=== log tail ==="
  tail -5 /tmp/orchard_run.log
  kill $SPID 2>/dev/null
  exit 1
fi

echo "=== ugv topics ==="
timeout 10 ign topic -l 2>/dev/null | grep ugv

echo "=== pose before ==="
timeout 10 ign topic -t /model/ugv_2marker/pose -e -n 1 2>&1 | head -30

timeout 10 ign topic -t /cmd_vel -m ignition.msgs.Twist -p "linear: {x: 0.4}, angular: {z: 0.2}" 2>&1
sleep 6

echo "=== pose after 6s drive ==="
timeout 10 ign topic -t /model/ugv_2marker/pose -e -n 1 2>&1 | head -30

echo "=== odometry sample ==="
timeout 10 ign topic -t /model/ugv_2marker/odometry -e -n 1 2>&1 | head -30

kill $SPID 2>/dev/null
sleep 1
echo "=== log errors ==="
grep -iE "error|failed" /tmp/orchard_run.log | head -8 || echo "(none)"
echo "RESULT: done"
