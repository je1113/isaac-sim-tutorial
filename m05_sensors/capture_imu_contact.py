from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.sensors.physics import IMUSensor, ContactSensor

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))
cube = world.scene.add(SingleRigidPrim(prim_path="/World/Cube", name="cube"))
world.reset()

# --- 함정 재현: /World/franka는 여러 링크를 담는 컨테이너(articulation root)일 뿐,
# 실제 관성/충돌을 가진 물리 바디가 아니다. 여기에 IMU를 붙이면 값이 절대 안 바뀐다.
imu_wrong = IMUSensor(prim_path="/World/franka/imu_wrong")
# --- 올바른 부착: 실제로 움직이는 링크(panda_hand)의 자식으로.
imu_correct = IMUSensor(prim_path="/World/franka/panda_hand/imu_correct")

# 접촉 센서: 그리퍼 손가락(실제 충돌 지오메트리가 있는 링크)에 부착
contact_left = ContactSensor(prim_path="/World/franka/panda_leftfinger/contact")
contact_right = ContactSensor(prim_path="/World/franka/panda_rightfinger/contact")

world.reset()  # 센서 프림 추가 후 재-리셋해서 물리 핸들 갱신

for _ in range(10):
    world.step(render=False)

print("=== 팔이 정지 상태일 때 (컨테이너 vs 실제 바디) ===")
print("imu_wrong (컨테이너 Xform):", imu_wrong.get_current_frame())
print("imu_correct (panda_hand):", imu_correct.get_current_frame())

controller = PickPlaceController(name="pick_place", gripper=franka.gripper, robot_articulation=franka)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

cube_pos_start, _ = cube.get_world_pose()
place_position = cube_pos_start + np.array([0.0, 0.3, 0.0])

log = []
max_force_left = 0.0
max_force_right = 0.0
any_contact = False
step = 0
while not controller.is_done() and step < 2000:
    actions = controller.forward(
        picking_position=cube_pos_start,
        placing_position=place_position,
        current_joint_positions=franka.get_joint_positions(),
    )
    franka.apply_action(actions)
    world.step(render=False)

    cl = contact_left.get_current_frame()
    cr = contact_right.get_current_frame()
    max_force_left = max(max_force_left, cl["force"])
    max_force_right = max(max_force_right, cr["force"])
    if cl["in_contact"] or cr["in_contact"]:
        any_contact = True

    if step % 10 == 0:
        wrong_frame = imu_wrong.get_current_frame()
        correct_frame = imu_correct.get_current_frame()
        log.append({
            "step": step,
            "event": controller.get_current_event(),
            "imu_wrong_ang_vel": wrong_frame["ang_vel"].tolist(),
            "imu_correct_ang_vel": correct_frame["ang_vel"].tolist(),
            "contact_left_in_contact": cl["in_contact"],
            "contact_left_force": cl["force"],
            "contact_right_in_contact": cr["in_contact"],
            "contact_right_force": cr["force"],
        })
    step += 1

print("\n=== 그립-앤-플레이스 진행 중 10스텝마다 로그 ===")
for entry in log:
    print(entry)

print("\n큐브를 잡는 동안 접촉이 한 번이라도 감지됐는가:", any_contact)
print("left/right 최대 접촉힘:", max_force_left, max_force_right)

simulation_app.close()
