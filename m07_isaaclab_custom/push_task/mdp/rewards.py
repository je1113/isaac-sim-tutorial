from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _target_pos_w(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Resolve the commanded target position (robot-root frame) into the world frame."""
    robot = env.scene["robot"]
    command = env.command_manager.get_command(command_name)
    des_pos_w, _ = combine_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, command[:, :3])
    return des_pos_w


def cube_to_target_distance(
    env: ManagerBasedRLEnv, command_name: str, cube_cfg: SceneEntityCfg = SceneEntityCfg("cube")
) -> torch.Tensor:
    """Raw L2 distance between the cube and its target position (penalty, use a negative weight)."""
    cube: RigidObject = env.scene[cube_cfg.name]
    des_pos_w = _target_pos_w(env, command_name)
    return torch.norm(cube.data.root_pos_w - des_pos_w, dim=1)


def cube_to_target_distance_tanh(
    env: ManagerBasedRLEnv, std: float, command_name: str, cube_cfg: SceneEntityCfg = SceneEntityCfg("cube")
) -> torch.Tensor:
    """Tanh-shaped fine-grained reward that saturates as the cube nears the target."""
    cube: RigidObject = env.scene[cube_cfg.name]
    des_pos_w = _target_pos_w(env, command_name)
    distance = torch.norm(cube.data.root_pos_w - des_pos_w, dim=1)
    return 1 - torch.tanh(distance / std)


def ee_to_cube_distance(
    env: ManagerBasedRLEnv,
    std: float,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot", body_names="panda_hand"),
    cube_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Shaping reward that encourages the end-effector to approach the cube before it can push it."""
    robot = env.scene[robot_cfg.name]
    cube: RigidObject = env.scene[cube_cfg.name]
    ee_pos_w = robot.data.body_pos_w[:, robot_cfg.body_ids[0]]
    distance = torch.norm(cube.data.root_pos_w - ee_pos_w, dim=1)
    return 1 - torch.tanh(distance / std)
