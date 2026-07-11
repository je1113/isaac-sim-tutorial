from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import time
import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"
COUNTDOWN_SECONDS = 15  # 이 시간 동안 뷰포트 각도를 잡고 Movie Capture를 켜세요

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))
cube = world.scene.add(SingleRigidPrim(prim_path="/World/Cube", name="cube"))
world.reset()

controller = PickPlaceController(name="pick_place", gripper=franka.gripper, robot_articulation=franka)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

cube_pos_start, _ = cube.get_world_pose()
place_position = cube_pos_start + np.array([0.0, 0.3, 0.0])

print(f"\n=== {COUNTDOWN_SECONDS}초 카운트다운 시작 ===")
print("지금 뷰포트에서 마우스로 카메라 각도를 잡고, Window > Movie Capture를 열어 Record를 누르세요.\n")
start_time = time.time()
last_printed = COUNTDOWN_SECONDS + 1
while time.time() - start_time < COUNTDOWN_SECONDS:
    remaining = int(COUNTDOWN_SECONDS - (time.time() - start_time))
    if remaining != last_printed:
        print(f"  {remaining}...")
        last_printed = remaining
    world.step(render=True)

print("\n=== pick-and-place 시작 ===\n")
step = 0
while not controller.is_done() and step < 2000:
    actions = controller.forward(
        picking_position=cube_pos_start,
        placing_position=place_position,
        current_joint_positions=franka.get_joint_positions(),
    )
    franka.apply_action(actions)
    world.step(render=True)
    step += 1

cube_pos_end, _ = cube.get_world_pose()
print("\n=== 완료 ===")
print("총 스텝:", step, "/ 최종 큐브 위치:", cube_pos_end)
print("Movie Capture에서 Stop을 누르고 저장하세요. 이 창은 계속 열려있으니 Ctrl+C로 종료하세요.\n")

# 창을 유지해서 Movie Capture 저장/확인 시간을 준다
while simulation_app.is_running():
    world.step(render=True)
