"""生成 Gazebo world，几何参数逐项对齐 run_study.py。

用途：让 Gazebo 独立渲染同一组几何，验证 pair_probability() 里
"光线-椭球透射率" 这个简化假设是否与真实遮挡一致。

所有常量都从 run_study.py 抄来，不另行设定，避免两套几何漂移。
若 run_study.py 改了参数，这里必须同步改。
"""

import json
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------- 与 run_study.py 对齐
ROWS_Y = (-14.0, -10.0, -6.0, -2.0, 2.0, 6.0, 10.0, 14.0)
X_MIN, X_MAX, X_STEP = -18.0, 50.0, 2.0
TREE_Z = 2.0
JITTER = 0.15                          # orchard(): rng.normal(0, 0.15, ...)
CROWN_AXES = (1.2, 1.2, 1.5)           # pair_probability(): radius = [1.2, CROWN_B, 1.5]
MARKER_HALF_BASELINE = 0.5             # markers(): half_baseline = 0.5 * unit  -> 基线 1.0 m
MARKER_Z = 0.3                         # pair_probability(): start 的 z
CAM_Z = 6.0                            # 对齐 pair_probability() 里光线终点的 z（end 用 6.0）
CAM_Z_FOV = 5.7                        # HALF_FOV_RADIUS = 5.7 * tan(60 deg)，模型此处与 6.0 不一致
CAM_HALF_FOV_DEG = 60.0                # 上式里的 60 度是半视场
RADII = (2.0, 3.0, 4.5, 6.5, 9.0, 12.0)
ANGLES_DEG = tuple(range(0, 360, 45))
LOCAL_RADIUS = 15.0                    # 只保留标记物附近的树，远处不影响遮挡


def orchard(seed=0):
    """复现 run_study.orchard()：同一随机种子下树位应逐棵一致。"""
    rng = np.random.default_rng(10000 + seed)
    x, y = np.meshgrid(np.arange(X_MIN, X_MAX + 0.01, X_STEP), ROWS_Y)
    xy = np.stack((x.ravel(), y.ravel()), axis=-1) + rng.normal(0, JITTER, (x.size, 2))
    return np.column_stack((xy, np.full(x.size, TREE_Z)))


def sites():
    """固定站位：8 方位 x 6 半径，对应 run_scene() 里的 fixed 策略。"""
    out = []
    for ang in ANGLES_DEG:
        th = np.deg2rad(ang)
        for r in RADII:
            out.append({
                "angle_deg": ang, "radius": r,
                "x": r * np.cos(th), "y": r * np.sin(th),
            })
    return out


def cam_pose(x, y):
    """站位 -> (x, y, z, roll, pitch, yaw)，朝向标记物中点 (0,0,MARKER_Z)。"""
    horiz = float(np.hypot(x, y))
    yaw = float(np.arctan2(-y, -x))
    # Gazebo 中 pitch 绕 Y 轴正转把 +X 压向 -Z，即正值才低头；负值会抬头看天
    pitch = float(np.arctan2(CAM_Z - MARKER_Z, max(horiz, 1e-6)))
    return x, y, CAM_Z, 0.0, pitch, yaw


def crown_sdf(idx, cx, cy, cz):
    ax, ay, az = CROWN_AXES
    # 冠层用深绿、无光泽，避免高光干扰标记物检测
    return f"""
    <model name="crown_{idx}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><ellipsoid><radii>{ax} {ay} {az}</radii></ellipsoid></geometry>
          <material>
            <ambient>0.05 0.28 0.05 1</ambient>
            <diffuse>0.05 0.28 0.05 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def marker_sdf(name, x, y):
    # 自发光纯品红：不受光照与材质影响，便于按颜色阈值检测
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {MARKER_Z:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><sphere><radius>0.09</radius></sphere></geometry>
          <material>
            <ambient>1 0 1 1</ambient>
            <diffuse>1 0 1 1</diffuse>
            <specular>0 0 0 1</specular>
            <emissive>1 0 1 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""


def build(seed=0, render_engine="ogre2"):
    trees = orchard(seed)
    keep = np.hypot(trees[:, 0], trees[:, 1]) <= LOCAL_RADIUS
    local = trees[keep]

    crowns = "".join(crown_sdf(i, t[0], t[1], t[2]) for i, t in enumerate(local))
    markers = (marker_sdf("marker_0", -MARKER_HALF_BASELINE, 0.0)
               + marker_sdf("marker_1", MARKER_HALF_BASELINE, 0.0))

    s0 = sites()[0]
    px, py, pz, rr, pp, yy = cam_pose(s0["x"], s0["y"])

    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="orchard">

    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>{render_engine}</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- 自定义 world 不会自动加载 UserCommands；缺了它就没有 /world/<name>/set_pose，
         只能靠重启 Gazebo 换站位，48 个站位会慢十倍 -->
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.95 0.95 0.95 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <direction>-0.25 0.25 -1</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.30 0.26 0.20 1</ambient>
            <diffuse>0.30 0.26 0.20 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>
{crowns}
{markers}
    <model name="uav">
      <static>true</static>
      <pose>{px:.3f} {py:.3f} {pz:.3f} {rr:.4f} {pp:.4f} {yy:.4f}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <pose>0 0 0 0 0 0</pose>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>false</visualize>
          <camera>
            <horizontal_fov>{np.deg2rad(2 * CAM_HALF_FOV_DEG):.6f}</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>200</far></clip>
          </camera>
          <topic>cam/image</topic>
        </sensor>
      </link>
    </model>

  </world>
</sdf>
"""


def main():
    out = Path(__file__).resolve().parent
    world = out / "orchard.sdf"
    world.write_text(build(), encoding="utf-8")

    plan = []
    for s in sites():
        px, py, pz, rr, pp, yy = cam_pose(s["x"], s["y"])
        plan.append(dict(s, pose=[round(px, 4), round(py, 4), round(pz, 4),
                                  round(rr, 6), round(pp, 6), round(yy, 6)],
                         dist3d=round(float(np.hypot(np.hypot(s["x"], s["y"]),
                                                     CAM_Z - MARKER_Z)), 4)))
    (out / "sites.json").write_text(json.dumps(plan, indent=1), encoding="utf-8")

    trees = orchard(0)
    local = trees[np.hypot(trees[:, 0], trees[:, 1]) <= LOCAL_RADIUS]
    print("world:", world)
    print("树总数 %d，保留 %.0f m 内 %d 棵" % (len(trees), LOCAL_RADIUS, len(local)))
    print("站位 %d 个（%d 方位 x %d 半径）" % (len(plan), len(ANGLES_DEG), len(RADII)))
    print("相机高度 %.1f m，全视场 %.0f deg，标记基线 %.1f m"
          % (CAM_Z, 2 * CAM_HALF_FOV_DEG, 2 * MARKER_HALF_BASELINE))


if __name__ == "__main__":
    main()
