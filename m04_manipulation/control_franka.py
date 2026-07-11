from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationActions

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(Articulation(prim_paths_expr="/World/franka", name="franka"))
world.reset()

start = franka.get_joint_positions()[0]          # shape (9,)
target = start.copy()
target[:7] += 0.3                                # 팔 7축만 +0.3 rad, 그리퍼 2축은 그대로 유지

for step in range(200):
    franka.apply_action(ArticulationActions(joint_positions=target))
    world.step(render=False)
    if step in (0, 49, 99, 149, 199):
        current = franka.get_joint_positions()[0]
        err = np.abs(current - target)
        print(f"step {step:3d} | current: {np.round(current, 4)} | max_err: {err.max():.4f}")

simulation_app.close()
