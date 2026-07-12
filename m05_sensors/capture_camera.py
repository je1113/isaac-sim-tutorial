from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from pxr import Usd
from isaacsim.core.api import World
from isaacsim.core.utils.semantics import add_labels, get_labels
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.sensors.camera import Camera
from PIL import Image

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"
OUT_DIR = "/home/pw/Documents/isaacsim/practicum/m05_sensors"

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

# instance segmentation이 의미있는 ID를 가지려면 prim에 semantic label이 있어야 함
stage = get_current_stage()
franka_prim = stage.GetPrimAtPath("/World/franka")
cube_prim = stage.GetPrimAtPath("/World/Cube")

# 프랑카의 각 링크 geometry는 instanceable reference라, /World/franka에 붙인 라벨이
# USD 인스턴스 경계를 넘어 실제 메시까지 전파되지 않는다 (prototype 공유 구조라
# 조상 prim의 로컬 opinion이 안쪽으로 안 들어감) -> instanceable을 꺼서 우회
instanceable_count = 0
for prim in Usd.PrimRange(franka_prim):
    if prim.IsInstanceable():
        prim.SetInstanceable(False)
        instanceable_count += 1
print(f"instanceable 해제한 프림 수: {instanceable_count}")

add_labels(franka_prim, ["franka"], instance_name="class")
add_labels(cube_prim, ["cube"], instance_name="class")
print("franka labels:", get_labels(franka_prim))
print("cube labels:", get_labels(cube_prim))

# 씬을 내려다보는 위치에 카메라 배치 (프랑카+큐브가 보이도록)
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([1.5, 1.5, 1.5]),
    frequency=20,
    resolution=(640, 480),
)
camera.initialize()
camera.set_focal_length(1.5)
camera.add_distance_to_image_plane_to_frame()
camera.add_instance_segmentation_to_frame(init_params={"colorize": True})

# 큐브를 바라보도록 카메라를 회전
# lookat_to_quatf(camera, target, up)는 로컬 -Z가 정면인 USD 카메라 컨벤션 기준으로
# "camera 인자 -> target 인자" 방향을 로컬 +Z로 잡는다. 실제로 eye에서 target을 바라보게
# 하려면 인자를 (target, eye, up) 순서로 뒤집어 넣어야 한다 (isaacsim 내부 camera_utils.py의
# CameraFollow가 쓰는 것과 동일한 패턴).
from pxr import Gf
from isaacsim.core.utils.rotations import lookat_to_quatf

target = Gf.Vec3d(0.5, 0.0, 0.1)
eye = Gf.Vec3d(1.5, 1.5, 1.5)
up = Gf.Vec3d(0.0, 0.0, 1.0)
quat = lookat_to_quatf(target, eye, up)
orientation = np.array([quat.GetReal(), *quat.GetImaginary()])  # (w, x, y, z)
camera.set_world_pose(position=np.array([eye[0], eye[1], eye[2]]), orientation=orientation, camera_axes="usd")

# RTX 렌더러 워밍업 (초기 프레임은 어둡거나 노이즈가 있음)
for _ in range(60):
    world.step(render=True)

rgb = camera.get_rgba()
depth = camera.get_depth()
seg = camera.get_current_frame()["instance_segmentation"]
if isinstance(seg, dict):  # colorize=False 기본값이면 순수 np.array, 버전에 따라 dict일 수도 있어 방어
    seg = seg["data"]

unique_colors = np.unique(seg.reshape(-1, seg.shape[-1]), axis=0)
print("rgb shape:", rgb.shape, "dtype:", rgb.dtype, "min/max:", rgb.min(), rgb.max())
print("depth shape:", depth.shape, "dtype:", depth.dtype, "min/max:", np.nanmin(depth), np.nanmax(depth))
print("seg shape:", seg.shape, "dtype:", seg.dtype, "unique colors (colorize=True):", len(unique_colors))

Image.fromarray(rgb[:, :, :3]).save(f"{OUT_DIR}/rgb.png")

depth_clipped = np.nan_to_num(np.squeeze(depth), nan=0.0, posinf=0.0)
depth_norm = (depth_clipped / (depth_clipped.max() + 1e-6) * 255).astype(np.uint8)
Image.fromarray(depth_norm).save(f"{OUT_DIR}/depth.png")

Image.fromarray(seg[:, :, :3]).save(f"{OUT_DIR}/segmentation.png")

print("저장 완료:", OUT_DIR)

simulation_app.close()
