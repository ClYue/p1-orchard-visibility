# -*- coding: utf-8 -*-
"""
M3 果园 SDF 世界生成器（Gazebo Fortress）
=========================================
从 E1 的同一套几何参数（直接 import e1_occlusion_calibration.CFG）生成
worlds/orchard.world：
- 地面 + 平行光；
- 树行：树干圆柱（r=0.1, h=0.6）+ 冠层椭球（单位球 STL 按 (a,b,c) 缩放，
  中心高 canopy_zc），树位抖动 seed 固定可复现；
- 内联 UGV（model://ugv_2marker）。

简化声明：冠层用单位球 STL 缩放成精确椭球，几何与 E1 一致；
树干遮挡未建（E1 口径：6 m 高的视线不过树干，此处树干仅为视觉/物理完整）。

用法：
  python gen_orchard.py
输出：
  ../meshes/ellipsoid.stl        （经 models/p1_assets/meshes/ 落盘）
  ../worlds/orchard.world

运行（无头）：
  IGN_GAZEBO_RESOURCE_PATH=<仓库>/models \
    ign gazebo -s -r -N worlds/orchard.world
"""
import sys
import numpy as np
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[1] / "tools"))  # 08_Gazebo仿真/tools
from e1_occlusion_calibration import CFG  # noqa: E402  单一几何来源

SEED = 42  # 世界生成专用种子（与 E1 主标定的 0..19 独立）


def build_tree_centers(cfg, rng):
    n = cfg["n_rows_each_side"]
    rows = (np.arange(2 * n) - (2 * n - 1) / 2.0) * cfg["row_spacing"]
    cols = np.arange(-cfg["x_extent"], cfg["x_extent"] + 1e-9, cfg["tree_spacing"])
    cx, cy = np.meshgrid(cols, rows)
    centers = np.stack(
        [cx.ravel() + rng.normal(0, cfg["jitter_std"], cx.size),
         cy.ravel() + rng.normal(0, cfg["jitter_std"], cx.size),
         np.full(cx.size, cfg["canopy_zc"])],
        axis=1,
    )
    return centers


def write_unit_sphere_stl(path, rings=10, segments=14):
    """单位球 ASCII STL（顶点在单位球面，实例化时按 (a,b,c) 缩放）。"""
    th = np.linspace(0, np.pi, rings + 1)
    ph = np.linspace(0, 2 * np.pi, segments + 1)
    pts = np.empty((rings + 1, segments + 1, 3))
    for i, t in enumerate(th):
        for j, p in enumerate(ph):
            pts[i, j] = [np.sin(t) * np.cos(p), np.sin(t) * np.sin(p), np.cos(t)]
    lines = ["solid ellipsoid"]
    def tri(a, b, c):
        n = np.cross(b - a, c - a)
        nn = n / (np.linalg.norm(n) + 1e-12)
        lines.append(" facet normal %.4e %.4e %.4e" % tuple(nn))
        lines.append("  outer loop")
        for v in (a, b, c):
            lines.append("   vertex %.4e %.4e %.4e" % tuple(v))
        lines.append("  endloop")
        lines.append(" endfacet")
    for i in range(rings):
        for j in range(segments):
            p00, p01 = pts[i, j], pts[i, j + 1]
            p10, p11 = pts[i + 1, j], pts[i + 1, j + 1]
            if i > 0:
                tri(p00, p11, p01)
            if i < rings - 1:
                tri(p00, p10, p11)
    lines.append("endsolid ellipsoid")
    Path(path).write_text("\n".join(lines), encoding="ascii")


def tree_sdf(idx, c, cfg):
    x, y, zc = c
    a, b, cc = cfg["canopy_a"], cfg["canopy_b"], cfg["canopy_c"]
    trunk_h = cfg["canopy_zc"] - cfg["canopy_c"]  # 0.5 m
    return f"""
  <model name="tree_{idx}">
    <static>true</static>
    <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
    <link name="trunk">
      <pose>0 0 {trunk_h / 2:.3f} 0 0 0</pose>
      <collision name="trunk_col">
        <geometry><cylinder><radius>0.1</radius><length>{trunk_h:.3f}</length></cylinder></geometry>
      </collision>
      <visual name="trunk_vis">
        <geometry><cylinder><radius>0.1</radius><length>{trunk_h:.3f}</length></cylinder></geometry>
        <material><ambient>0.45 0.30 0.18 1</ambient><diffuse>0.55 0.37 0.22 1</diffuse></material>
      </visual>
    </link>
    <link name="canopy">
      <pose>0 0 {zc:.3f} 0 0 0</pose>
      <collision name="canopy_col">
        <geometry><mesh><uri>model://p1_assets/meshes/ellipsoid.stl</uri>
          <scale>{a:.3f} {b:.3f} {cc:.3f}</scale></mesh></geometry>
      </collision>
      <visual name="canopy_vis">
        <geometry><mesh><uri>model://p1_assets/meshes/ellipsoid.stl</uri>
          <scale>{a:.3f} {b:.3f} {cc:.3f}</scale></mesh></geometry>
        <material><ambient>0.10 0.35 0.12 1</ambient><diffuse>0.15 0.50 0.18 1</diffuse></material>
      </visual>
    </link>
  </model>"""


def world_sdf(trees_xml, cfg):
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="orchard">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8066</gravity>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.4 0.4 0.4 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.55 0.62 0.45 1</ambient><diffuse>0.65 0.72 0.52 1</diffuse></material>
        </visual>
      </link>
    </model>
{trees_xml}

    <include>
      <uri>model://ugv_2marker</uri>
      <name>ugv_2marker</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>"""


def main():
    root = Path(__file__).resolve().parent.parent     # p1_gz_sim/
    mesh_dir = root / "models" / "p1_assets" / "meshes"
    world_dir = root / "worlds"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    world_dir.mkdir(parents=True, exist_ok=True)

    stl_path = mesh_dir / "ellipsoid.stl"
    if not stl_path.exists():
        write_unit_sphere_stl(stl_path)
        print(f"[gen] mesh  -> {stl_path}")

    cfg = dict(CFG)
    trees = build_tree_centers(cfg, np.random.default_rng(SEED))
    trees_xml = "\n".join(tree_sdf(i, c, cfg) for i, c in enumerate(trees))
    world_path = world_dir / "orchard.world"
    world_path.write_text(world_sdf(trees_xml, cfg), encoding="utf-8")
    print(f"[gen] world -> {world_path}  ({len(trees)} trees, "
          f"row_spacing={cfg['row_spacing']} m, canopy={cfg['canopy_a']}x{cfg['canopy_b']}x{cfg['canopy_c']} m)")
    print(f"[gen] run with: IGN_GAZEBO_RESOURCE_PATH={root / 'models'} "
          f"ign gazebo -s -r -N {world_path}")


if __name__ == "__main__":
    main()
