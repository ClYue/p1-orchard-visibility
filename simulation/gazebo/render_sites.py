"""逐站位渲染：SDF 写死朝向 + ign topic --json-output 取图，不依赖 ROS。

两个教训写在这里：
1) 不要用 /world/<w>/set_pose 移动相机——实测它只更新 static 模型的 position，
   orientation 不生效，135 度站位拍到了初始朝向的树冠背面。
2) 不要用 ros_gz_bridge 取图——bridge 启动慢，且订阅里会残留上一站的旧帧，
   表现为多个站位 mean 完全相同。改用 ign topic --json-output，每站独立取一帧。
"""

import argparse
import base64
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image as PImage

import make_world as M


def cleanup():
    """ign 是 ruby 启动器，kill 父进程杀不掉真正的 Gazebo 子进程。"""
    for pat in ("ignition-gazebo", "ign gazebo"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.6)


def wait_topic(deadline=40):
    t0 = time.time()
    while time.time() - t0 < deadline:
        r = subprocess.run(["ign", "topic", "-l"], capture_output=True, text=True)
        if "cam/image" in r.stdout:
            return True
        time.sleep(0.8)
    return False


def grab(timeout=60):
    """取一帧，返回 HxWx3 的 RGB 数组。"""
    r = subprocess.run(["ign", "topic", "-t", "/cam/image", "-e", "-n", "1",
                        "--json-output"], capture_output=True, text=True, timeout=timeout)
    d = json.loads(r.stdout)
    w, h = int(d["width"]), int(d["height"])
    raw = base64.b64decode(d["data"])
    img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/shots")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--engine", default="ogre", help="WSL 无 N 卡时用 ogre；有 N 卡可试 ogre2")
    ap.add_argument("--only", default="", help="只跑指定站位，形如 2:0,2:135,4.5:45")
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    sites = json.loads((here / "sites.json").read_text(encoding="utf-8"))
    if args.only:
        want = set()
        for part in args.only.split(","):
            r, a = part.split(":")
            want.add((float(r), int(a)))
        sites = [s for s in sites if (float(s["radius"]), int(s["angle_deg"])) in want]
        if not sites:
            print("--only 未匹配到任何站位")
            sys.exit(1)
    if args.limit:
        sites = sites[:args.limit]

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    base = M.build(render_engine=args.engine)

    done = 0
    for i, s in enumerate(sites):
        tag = "r%05.1f_a%03d" % (s["radius"], s["angle_deg"])
        pose_txt = " ".join("%.5f" % v for v in M.cam_pose(s["x"], s["y"]))
        sdf = re.sub(r'(<model name="uav">\s*<static>true</static>\s*<pose>)[^<]+(</pose>)',
                     lambda m: m.group(1) + pose_txt + m.group(2), base, count=1)
        Path("/tmp/site_current.sdf").write_text(sdf, encoding="utf-8")

        cleanup()
        subprocess.Popen(["ign", "gazebo", "-s", "-r", "--headless-rendering",
                          "-v", "1", "/tmp/site_current.sdf"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if not wait_topic():
            print("[%02d] %s topic 未就绪" % (i, tag), flush=True)
            continue
        time.sleep(1.5)          # 跳过尚未稳定的第一帧

        try:
            img = grab()
        except Exception as e:
            print("[%02d] %s 取图失败: %s" % (i, tag, str(e)[:60]), flush=True)
            continue

        PImage.fromarray(img).save(out / (tag + ".png"))
        done += 1
        print("[%02d] %s mean=%.1f" % (i, tag, img.mean()), flush=True)

    cleanup()
    print("完成 %d / %d" % (done, len(sites)))


if __name__ == "__main__":
    main()
