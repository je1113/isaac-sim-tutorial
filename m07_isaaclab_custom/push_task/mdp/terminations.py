from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def cube_reached_target(
    env: ManagerBasedRLEnv,
    command_name: str,
    threshold: float = 0.03,
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Early-terminate (success) once the cube is within `threshold` meters of the target."""
    robot = env.scene["robot"]
    cube: RigidObject = env.scene[cube_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_pos_w, _ = combine_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, command[:, :3])
    distance = torch.norm(cube.data.root_pos_w - des_pos_w, dim=1)
    return distance < threshold
