from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
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
world.reset()

kinematics_solver = LulaKinematicsSolver(robot_description_path=ROBOT_DESCRIPTION_YAML, urdf_path=FRANKA_URDF)
print("사용 가능한 frame 목록:", kinematics_solver.get_all_frame_names())

articulation_ik = ArticulationKinematicsSolver(
    robot_articulation=franka, kinematics_solver=kinematics_solver, end_effector_frame_name="right_gripper"
)

# 1) 시작 pose (FK)
start_pos, start_rot = articulation_ik.compute_end_effector_pose()
print("시작 엔드이펙터 위치:", start_pos)

# 2) 큐브(0.5, 0, 0.05, scale 0.05) 위 approach pose로 IK 계산 (위치만, 방향은 무시)
target_position = np.array([0.5, 0.0, 0.25])
ik_action, success = articulation_ik.compute_inverse_kinematics(target_position=target_position)
print("IK 성공 여부:", success)
print("IK 결과 joint_positions:", ik_action.joint_positions)

# 3) 실제 로봇에 적용하고 여러 스텝 진행
franka.apply_action(ik_action)
for _ in range(120):
    world.step(render=False)

# 4) 도달 후 FK로 실제 엔드이펙터 위치 재확인
final_pos, final_rot = articulation_ik.compute_end_effector_pose()
print("목표 위치:", target_position)
print("실제 도달 위치(FK):", final_pos)
print("위치 오차:", np.linalg.norm(final_pos - target_position))

simulation_app.close()
