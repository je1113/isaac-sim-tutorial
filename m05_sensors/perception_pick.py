from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from pxr import Gf, UsdGeom
from PIL import Image

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.rotations import lookat_to_quatf
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.sensors.camera import Camera
from isaacsim.sensors.physics import ContactSensor

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"
OUT_DIR = "/home/pw/Documents/isaacsim/practicum/m05_sensors"

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))
cube = world.scene.add(SingleRigidPrim(prim_path="/World/Cube", name="cube"))
world.reset()

# 색 마스크로 찾기 쉽게 큐브를 눈에 띄는 빨간색으로 지정 (기본은 무채색이라 배경/로봇과 구분이 애매함)
cube_geom = UsdGeom.Gprim(cube.prim)
cube_geom.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.1, 0.1)])

contact_left = ContactSensor(prim_path="/World/franka/panda_leftfinger/contact")
contact_right = ContactSensor(prim_path="/World/franka/panda_rightfinger/contact")

camera = Camera(prim_path="/World/Camera", position=np.array([1.5, 1.5, 1.5]), resolution=(640, 480))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()

target = Gf.Vec3d(0.4, 0.0, 0.15)
eye = Gf.Vec3d(1.5, 1.5, 1.5)
up = Gf.Vec3d(0.0, 0.0, 1.0)
quat = lookat_to_quatf(target, eye, up)
orientation = np.array([quat.GetReal(), *quat.GetImaginary()])
camera.set_world_pose(position=np.array([eye[0], eye[1], eye[2]]), orientation=orientation, camera_axes="usd")

world.reset()
for _ in range(30):
    world.step(render=True)

# --- 인식 단계: RGB에서 빨간 마스크 중심점을 찾고, depth로 3D 위치 추정 ---
rgb = camera.get_rgba()[:, :, :3].astype(np.int16)
depth = np.squeeze(camera.get_depth())

r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
red_mask = (r > 120) & (r - g > 40) & (r - b > 40)
print("빨간 마스크 픽셀 수:", red_mask.sum())

Image.fromarray((red_mask * 255).astype(np.uint8)).save(f"{OUT_DIR}/perception_mask.png")

ys, xs = np.nonzero(red_mask)
centroid_u = float(xs.mean())
centroid_v = float(ys.mean())
centroid_depth = float(np.median(depth[ys, xs]))
print(f"마스크 중심 픽셀: (u={centroid_u:.1f}, v={centroid_v:.1f}), depth={centroid_depth:.3f}m")

points_2d = np.array([[centroid_u, centroid_v]])
depth_arr = np.array([centroid_depth])
estimated_world_point = camera.get_world_points_from_image_coords(points_2d, depth_arr)[0]
print("카메라 인식으로 추정한 큐브 위치:", estimated_world_point)

actual_cube_pos, _ = cube.get_world_pose()
print("실제(GT) 큐브 위치:", actual_cube_pos)
print("추정 오차(m):", np.linalg.norm(estimated_world_point - actual_cube_pos))

# --- pick-and-place는 추정 위치로 시도 (z만 큐브 절반 높이만큼 보정 - 마스크 중심점은
# 큐브 윗면 근처를 가리키기 쉬워서, 그리퍼가 물체 중심을 잡도록 GT의 z 오프셋 관례를 따름) ---
picking_position = np.array([estimated_world_point[0], estimated_world_point[1], actual_cube_pos[2]])
place_position = actual_cube_pos + np.array([0.0, 0.3, 0.0])

controller = PickPlaceController(name="pick_place", gripper=franka.gripper, robot_articulation=franka)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

step = 0
grasped_ever = False
while not controller.is_done() and step < 2000:
    actions = controller.forward(
        picking_position=picking_position,
        placing_position=place_position,
        current_joint_positions=franka.get_joint_positions(),
    )
    franka.apply_action(actions)
    world.step(render=False)

    cl = contact_left.get_current_frame()
    cr = contact_right.get_current_frame()
    if cl["in_contact"] and cr["in_contact"]:
        grasped_ever = True
    step += 1

final_cube_pos, _ = cube.get_world_pose()
print("\n=== 결과 ===")
print("총 스텝:", step)
print("그립 중 양쪽 손가락이 동시에 접촉한 적이 있는가:", grasped_ever)
print("최종 큐브 위치:", final_cube_pos, "(시작 위치 대비 이동:", final_cube_pos - actual_cube_pos, ")")

simulation_app.close()
