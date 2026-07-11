"""Module 02 practicum: standalone Python API control of a fresh robot asset
(Denso COBOTTA PRO 1300, placed via GUI in module 02's own robot_scene.usd —
independent of the ROS2-course biped used before).
"""
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m02_python_api/robot_scene.usd"
ROBOT_PRIM_PATH = "/World/cobotta_pro_1300"


def main():
    omni.usd.get_context().open_stage(STAGE_PATH)
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    robot = world.scene.add(Robot(prim_path=ROBOT_PRIM_PATH, name="cobotta"))
    world.reset()

    dof_names = robot.dof_names
    print(f"[m02] dof_names ({len(dof_names)}): {dof_names}")

    start = robot.get_joint_positions()
    print(f"[m02] start joint positions: {np.round(start, 3)}")

    # Modest fixed offset per joint (rad) — works regardless of this arm's DOF count,
    # and PhysX joint limits will simply clamp any joint that would go out of range.
    target = start + 0.3
    print(f"[m02] target joint positions: {np.round(target, 3)}")

    for step in range(200):
        robot.set_joint_positions(target)
        world.step(render=False)
        if step in (0, 49, 99, 149, 199):
            current = robot.get_joint_positions()
            err = np.abs(current - target)
            print(f"[m02] step {step:3d} | current: {np.round(current, 3)} | max_err: {err.max():.4f}")

    final = robot.get_joint_positions()
    final_err = np.abs(final - target)
    print(f"[m02] final max joint error: {final_err.max():.5f} rad")
    if final_err.max() < 0.01:
        print("[m02] CONVERGED: all joints within 0.01 rad of target")
    else:
        print("[m02] NOT CONVERGED — check drive gains / joint limits")

    simulation_app.close()


if __name__ == "__main__":
    main()
