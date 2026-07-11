from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))
cube = world.scene.add(SingleRigidPrim(prim_path="/World/Cube", name="cube"))
world.reset()

franka_base_pos, franka_base_rot = franka.get_world_pose()
print("franka 베이스 world pose - position:", franka_base_pos, "/ orientation:", franka_base_rot)

controller = PickPlaceController(name="pick_place", gripper=franka.gripper, robot_articulation=franka)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

cube_pos_start, _ = cube.get_world_pose()
place_position = cube_pos_start + np.array([0.0, 0.3, 0.0])  # Y방향으로 30cm 옆에 내려놓기
print("픽 위치:", cube_pos_start, "/ 플레이스 목표:", place_position)

max_height = cube_pos_start[2]
step = 0
last_event = -1
while not controller.is_done() and step < 2000:
    actions = controller.forward(
        picking_position=cube_pos_start,
        placing_position=place_position,
        current_joint_positions=franka.get_joint_positions(),
    )
    franka.apply_action(actions)
    world.step(render=False)
    cube_now, _ = cube.get_world_pose()
    max_height = max(max_height, cube_now[2])

    event = controller.get_current_event()
    if event != last_event:
        finger_pos = franka.gripper.get_joint_positions()
        ee_pos, _ = franka.end_effector.get_world_pose()
        print(
            f"  [step {step:4d}] event {last_event} -> {event} | finger: {finger_pos} | "
            f"ee: {ee_pos} | cube: {cube_now}"
        )
        last_event = event
    step += 1

cube_pos_end, _ = cube.get_world_pose()
print("총 스텝:", step)
print("집는 동안 도달한 최대 높이:", max_height, "(시작보다 높으면 성공적으로 들어올린 것)")
print("최종 큐브 위치:", cube_pos_end, "/ 목표였던 place 위치:", place_position)

simulation_app.close()
