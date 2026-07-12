"""Custom Isaac Lab task: Franka pushes a cube on a table to a target point.

Not part of the isaaclab_tasks package -- this is registered from outside IsaacLab's source
tree. Import this module once (before gym.make) to make the task ids available; see
train_push.py / play_push.py in this same directory.
"""

import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Push-Franka-Minimal-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.push_env_cfg:FrankaPushEnvCfg_Minimal",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:FrankaPushPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Push-Franka-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.push_env_cfg:FrankaPushEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:FrankaPushPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Push-Franka-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.push_env_cfg:FrankaPushEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:FrankaPushPPORunnerCfg",
    },
)
