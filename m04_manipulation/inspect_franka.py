from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

franka = world.scene.add(Articulation(prim_paths_expr="/World/franka", name="franka"))
world.reset()

print(f"dof_names ({len(franka.dof_names)}):", franka.dof_names)
print("현재 joint positions:", franka.get_joint_positions())

simulation_app.close()
