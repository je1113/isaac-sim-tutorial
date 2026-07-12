from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def cube_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Position of the pushed cube, expressed in the robot's root frame."""
    robot = env.scene[robot_cfg.name]
    cube: RigidObject = env.scene[cube_cfg.name]
    cube_pos_w = cube.data.root_pos_w[:, :3]
    cube_pos_b, _ = subtract_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, cube_pos_w)
    return cube_pos_b
