"""逐站位渲染：每站生成独立 SDF（朝向写死），不用 set_pose。

为什么不用 set_pose：
    实测 /world/orchard/set_pose 只更新了 static 模型的 position，
    orientation 未生效（135 度站位拍到了 SDF 初始朝向的树冠背面）。
    因此改为把每站的欧拉角直接写进 SDF，重新启动 Gazebo，保证朝向正确。
"""

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from PIL import Image as PImage
from rclpy.node import Node
from sensor_msgs.msg import Image

import make_world as M


class Cam(Node):
    def __init__(self):
        super().__init__("sdf_site_renderer")
        self.latest = None
        self.count = 0
        self.create_subscription(Image, "/cam/image", self.cb, 10)

    def cb(self, msg):
        h, w = msg.height, msg.width
        self.latest = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 3)
        self.count += 1


def cleanup():
    """ign 是 ruby 启动器，kill 父进程杀不掉真正的 Gazebo 子进程。

    不清理的话旧实例一直占着 /cam/image，后面取到的全是上一站的旧图，
    而且内存会一路涨到 Cannot allocate memory。
    """
    for pat in ("ignition-gazebo", "ign gazebo", "parameter_bridge"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def run(cmd, **kw):
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, **kw)


def wait_topic(deadline=45):
    t0 = time.time()
    while time.time() - t0 < deadline:
        r = subprocess.run(["ign", "topic", "-l"], capture_output=True, text=True)
        if "cam/image" in r.stdout:
            return True
        time.sleep(1.0)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/shots_sdf")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--engine", default="ogre")
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    sites = json.loads((here / "sites.json").read_text(encoding="utf-8"))
    if args.limit:
        sites = sites[:args.limit]

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    base = M.build(render_engine=args.engine)

    rclpy.init()
    done = 0

    for i, s in enumerate(sites):
        tag = "r%05.1f_a%03d" % (s["radius"], s["angle_deg"])
        pose = M.cam_pose(s["x"], s["y"])
        pose_txt = " ".join("%.5f" % v for v in pose)

        cleanup()                      # 先清掉上一站残留的 Gazebo / bridge

        # 替换 uav 的 pose（朝向随站位改变）
        sdf = re.sub(r'(<model name="uav">\s*<static>true</static>\s*<pose>)[^<]+(</pose>)',
                     lambda m: m.group(1) + pose_txt + m.group(2), base, count=1)
        tmp = Path("/tmp/site_current.sdf")
        tmp.write_text(sdf, encoding="utf-8")

        run(["ign", "gazebo", "-s", "-r", "--headless-rendering", "-v", "1", str(tmp)])
        if not wait_topic():
            print("[%02d] %s topic 未就绪" % (i, tag))
            cleanup()
            continue

        run(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
             "/cam/image@sensor_msgs/msg/Image@gz.msgs.Image"])
        time.sleep(4)

        # 每站新建节点：否则订阅里还留着上一站的缓存帧，会拿到重复的旧图
        node = Cam()
        t0 = time.time()
        while node.count < 4 and time.time() - t0 < 25:
            rclpy.spin_once(node, timeout_sec=0.3)

        if node.latest is not None:
            PImage.fromarray(node.latest).save(out / (tag + ".png"))
            done += 1
            print("[%02d] %s mean=%.1f" % (i, tag, node.latest.mean()), flush=True)
        else:
            print("[%02d] %s 未取到帧" % (i, tag), flush=True)

        node.destroy_node()
        cleanup()

    rclpy.shutdown()
    print("完成 %d / %d" % (done, len(sites)))


if __name__ == "__main__":
    main()
