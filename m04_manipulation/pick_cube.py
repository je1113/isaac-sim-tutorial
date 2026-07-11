from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"
ROBOT_DESCRIPTION_YAML = (
    "/home/pw/isaacsim_env/lib/python3.11/site-packages/isaacsim/exts/"
    "isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml"
)
FRANKA_URDF = (
    "/home/pw/isaacsim_env/lib/python3.11/site-packages/isaacsim/exts/"
    "isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf"
)

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(SingleArticulation(prim_path="/World/franka", name="franka"))
cube = world.scene.add(SingleRigidPrim(prim_path="/World/Cube", name="cube"))
world.reset()

kinematics_solver = LulaKinematicsSolver(robot_description_path=ROBOT_DESCRIPTION_YAML, urdf_path=FRANKA_URDF)
articulation_ik = ArticulationKinematicsSolver(
    robot_articulation=franka, kinematics_solver=kinematics_solver, end_effector_frame_name="right_gripper"
)


def move_arm_to(target_position, steps=120):
    ik_action, success = articulation_ik.compute_inverse_kinematics(target_position=target_position)
    if not success:
        print(f"  [경고] IK 실패: target={target_position}")
    franka.apply_action(ik_action)
    for _ in range(steps):
        world.step(render=False)
    return success


def set_gripper(width, steps=60):
    # panda_finger_joint1/2 인덱스 = 7, 8. 각 관절이 중심에서 width/2만큼 이동.
    target = np.array([width / 2.0, width / 2.0])
    franka.apply_action(ArticulationAction(joint_positions=target, joint_indices=np.array([7, 8])))
    for _ in range(steps):
        world.step(render=False)


def report_cube(label):
    pos, rot = cube.get_world_pose()
    print(f"   [{label}] 큐브 위치: {pos}, 회전(quat): {rot}")


cube_pos_start, cube_rot_start = cube.get_world_pose()
print("큐브 시작 위치:", cube_pos_start, "회전:", cube_rot_start)

print("1) 그리퍼 열기 (0.08 = 완전 개방)")
set_gripper(0.08)
report_cube("그리퍼 연 후")

print("2) 큐브 위 approach pose로 이동")
move_arm_to(np.array([0.5, 0.0, 0.25]))
report_cube("approach 후")

print("3) 큐브 높이로 하강 (grasp pose)")
move_arm_to(np.array([0.5, 0.0, cube_pos_start[2]]))
ee_pos, _ = articulation_ik.compute_end_effector_pose()
print("   하강 후 실제 엔드이펙터 위치(FK):", ee_pos, "(목표는 [0.5, 0.0,", cube_pos_start[2], "])")
report_cube("하강 후")

print("4) 그리퍼 닫기 (목표 0.0 -> 큐브에 막혀서 실제로는 큐브 폭 근처에서 멈춰야 정상)")
set_gripper(0.0)
finger_positions = franka.get_joint_positions()[7:9]
print("   닫은 후 실제 finger 위치:", finger_positions, "(0에 못 미치면 큐브에 막힌 것)")
report_cube("그리퍼 닫은 후")

print("5) 들어올리기")
move_arm_to(np.array([0.5, 0.0, 0.4]))

cube_pos_end, cube_rot_end = cube.get_world_pose()
print("큐브 최종 위치:", cube_pos_end, "회전:", cube_rot_end)
print("큐브가 들어올려진 높이:", cube_pos_end[2] - cube_pos_start[2])

simulation_app.close()
