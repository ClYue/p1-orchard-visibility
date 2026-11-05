# -*- coding: utf-8 -*-
"""
生成可视化世界 orchard_viz.world（仅用于论文图 1 渲染）
=====================================================
在 gen_orchard.py 的同一果园几何上叠加：
- 两个静态四旋翼视觉模型（通道内站位 / 跨行站位）；
- 四条视线（细圆柱）：绿色=可见，红色=被冠层遮挡；
- UGV 双标志板沿用 models/ugv_2marker。

⚠️ 飞行器是静态占位，无动力学、无传感器，不产出任何实验数据；
   图注必须写明 "visualisation only"。

用法：python gen_viz_world.py
输出：../worlds/orchard_viz.world（并打印两个站位的 p_vis 供核对）
"""
import math
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / "tools"))
from gen_orchard import CFG, SEED, build_tree_centers, tree_sdf  # noqa: E402
from e1_occlusion_calibration import (  # noqa: E402
    build_trees, path_length_in_ellipsoids,
)

# 两个站位（论文图 1 用）
ST_ALLEY = (2.0, 0.0, 6.0)     # 通道内、沿行方向 —— 视线不被遮挡
ST_ACROSS = (0.0, 4.0, 6.0)    # 跨行、越过邻行树冠 —— 视线被遮挡
MARKERS = [(0.5, 0.0, 0.3), (-0.5, 0.0, 0.3)]


def pvis_at(u, cfg, seed=0):
    """用 E1 同一套射线模型核对站位可见率。"""
    centers = build_trees(cfg, np.random.default_rng(seed))
    radii = np.array([cfg["canopy_a"], cfg["canopy_b"], cfg["canopy_c"]])
    vals = []
    for m in MARKERS:
        ell = path_length_in_ellipsoids(np.array(m), np.array([u]), centers, radii)[0]
        vals.append(math.exp(-cfg["extinction_k"] * ell))
    return float(np.mean(vals))


def line_xml(name, a, b, rgb, radius=0.012):
    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mx, my, mz = (ax + bx) / 2, (ay + by) / 2, (az + bz) / 2
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    r, g, bl = rgb
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{mx:.4f} {my:.4f} {mz:.4f} 0 {pitch:.5f} {yaw:.5f}</pose>
    <link name="l">
      <visual name="v">
        <geometry><cylinder><radius>{radius}</radius><length>{length:.4f}</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {bl} 1</ambient>
          <diffuse>{r} {g} {bl} 1</diffuse>
          <emissive>{r * 0.6:.3f} {g * 0.6:.3f} {bl * 0.6:.3f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>"""


def quad_xml(name, pose):
    x, y, z = pose
    return f"""
  <include>
    <uri>model://quad_visual</uri>
    <name>{name}</name>
    <pose>{x} {y} {z} 0 0 0.4</pose>
  </include>"""


def main():
    root = _HERE.parent
    cfg = dict(CFG)
    p_alley = pvis_at(ST_ALLEY, cfg)
    p_across = pvis_at(ST_ACROSS, cfg)
    print(f"[viz] p_vis @ alley station {ST_ALLEY} = {p_alley:.3f}")
    print(f"[viz] p_vis @ across station {ST_ACROSS} = {p_across:.3f}")

    trees = build_tree_centers(cfg, np.random.default_rng(SEED))
    trees_xml = "\n".join(tree_sdf(i, c, cfg) for i, c in enumerate(trees))

    extra = quad_xml("quad_alley", ST_ALLEY) + quad_xml("quad_across", ST_ACROSS)
    green = (0.15, 0.85, 0.35)
    red = (0.90, 0.20, 0.15)
    for i, m in enumerate(MARKERS, start=1):
        extra += line_xml(f"los_alley_{i}", ST_ALLEY, m, green)
        extra += line_xml(f"los_across_{i}", ST_ACROSS, m, red)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="orchard_viz">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8066</gravity>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.35 0.35 0.35 1</specular>
      <direction>-0.35 0.25 -0.90</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.30 0.33 0.38 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.5 -0.6 -0.6</direction>
    </light>

    <scene>
      <ambient>0.55 0.57 0.60 1</ambient>
      <background>0.72 0.79 0.86 1</background>
      <shadows>true</shadows>
    </scene>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.52 0.58 0.42 1</ambient><diffuse>0.60 0.66 0.47 1</diffuse></material>
        </visual>
      </link>
    </model>

{trees_xml}
{extra}

    <include>
      <uri>model://ugv_2marker</uri>
      <name>ugv_2marker</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>"""
    out = root / "worlds" / "orchard_viz.world"
    out.write_text(world, encoding="utf-8")
    print(f"[viz] world -> {out}")


if __name__ == "__main__":
    main()
