#!/bin/bash
# 启动 Gazebo + bridge，批量渲染站位，最后清理
# 用法：bash run_render.sh [限制数量] [输出目录]
set +e
source /opt/ros/humble/setup.bash

GDIR="/mnt/c/Users/yue_c/OneDrive - hnuahe.edu.cn/科研/论文/地空协同青基/p1-orchard-visibility/simulation/gazebo"
LIMIT=${1:-0}
OUT=${2:-/tmp/shots}
ENGINE=${3:-ogre}

cd "$GDIR" || exit 1
mkdir -p "$OUT"

if [ "$ENGINE" = "ogre" ]; then
  sed 's|<render_engine>ogre2</render_engine>|<render_engine>ogre</render_engine>|' orchard.sdf > /tmp/orchard_run.sdf
else
  cp orchard.sdf /tmp/orchard_run.sdf
fi
echo "引擎: $ENGINE | world: $(grep -c '<model name=' /tmp/orchard_run.sdf) 个模型"

ign gazebo -s -r --headless-rendering -v 2 /tmp/orchard_run.sdf > /tmp/gz_run.log 2>&1 &
GZ=$!
READY=0
for i in $(seq 1 40); do
  sleep 2
  if ign topic -l 2>/dev/null | grep -q cam/image; then READY=1; echo "topic 就绪 ${i}x2s"; break; fi
  kill -0 $GZ 2>/dev/null || { echo "Gazebo 退出"; break; }
done
if [ "$READY" != "1" ]; then
  echo "启动失败："; grep -iE "\[Err\]|Failed|error" /tmp/gz_run.log | head -6
  kill $GZ 2>/dev/null; exit 1
fi

ros2 run ros_gz_bridge parameter_bridge /cam/image@sensor_msgs/msg/Image@gz.msgs.Image > /tmp/br_run.log 2>&1 &
BR=$!
sleep 6

python3 render_sites.py --out "$OUT" --limit "$LIMIT"
RC=$?

kill $BR $GZ 2>/dev/null
sleep 1
kill -9 $GZ 2>/dev/null
echo "渲染退出码: $RC | 图片数: $(ls -1 "$OUT"/*.png 2>/dev/null | wc -l)"
