"""Custom manager-based task: push a cube on a table to a target point with a Franka arm.

Two variants are provided:
  - FrankaPushEnvCfg_Minimal: the first-pass design (module practicum step 3/4) -- a single
    distance reward, time_out-only termination, fixed cube start and fixed target. Meant to
    verify the pipeline trains at all before anything else is added.
  - FrankaPushEnvCfg: the full design (module practicum step 5) -- adds a reaching-shaping
    reward, action/velocity penalties, success + "cube fell off the table" terminations, and
    Event Manager randomization of the cube's start position and the target region.
"""

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import push_task.mdp as mdp  # noqa: E402

##
# Pre-defined configs
##
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip

##
# Scene definition
##


@configclass
class PushSceneCfg(InteractiveSceneCfg):
    """Table + Franka + a pushable cube."""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0), rot=(0.70711, 0.0, 0.0, 0.70711)),
    )

    robot: ArticulationCfg = MISSING

    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.45, 0.0, 0.055), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
            scale=(0.8, 0.8, 0.8),
            rigid_props=RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
        ),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP: where the cube should end up."""

    target_pos = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="panda_hand",
        resampling_time_range=(1.0e6, 1.0e6),  # sample once per episode, never mid-episode
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=MISSING,
            pos_y=MISSING,
            pos_z=(0.055, 0.055),
            roll=(0.0, 0.0),
            pitch=(0.0, 0.0),
            yaw=(0.0, 0.0),
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP: joint position control of the arm only (no gripper action)."""

    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=["panda_joint.*"], scale=0.5, use_default_offset=True
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        cube_position = ObsTerm(func=mdp.cube_position_in_robot_root_frame)
        target_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "target_pos"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events (reset-time randomization)."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (0.5, 1.5), "velocity_range": (0.0, 0.0)},
    )

    # empty pose_range = deterministic reset to the default position defined in PushSceneCfg.cube
    # (widened in FrankaPushEnvCfg to randomize the cube's start position -- see step 5)
    reset_cube = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {}, "velocity_range": {}, "asset_cfg": SceneEntityCfg("cube")},
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP. Starts as a single distance term (step 3); step 5 adds the rest."""

    cube_to_target = RewTerm(func=mdp.cube_to_target_distance, weight=-1.0, params={"command_name": "target_pos"})

    # -- added in FrankaPushEnvCfg (step 5), left as None (disabled) in the Minimal variant --
    cube_to_target_fine: RewTerm | None = RewTerm(
        func=mdp.cube_to_target_distance_tanh, weight=1.0, params={"std": 0.1, "command_name": "target_pos"}
    )
    ee_to_cube: RewTerm | None = RewTerm(
        func=mdp.ee_to_cube_distance,
        weight=0.5,
        params={"std": 0.25, "robot_cfg": SceneEntityCfg("robot", body_names="panda_hand")},
    )
    action_rate: RewTerm | None = RewTerm(func=mdp.action_rate_l2, weight=-1.0e-4)
    joint_vel: RewTerm | None = RewTerm(
        func=mdp.joint_vel_l2, weight=-1.0e-4, params={"asset_cfg": SceneEntityCfg("robot")}
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP. Minimal variant keeps only time_out (matches the built-in
    reach example's design gap noted in step 1); the full variant adds success + failure signals."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # -- added in FrankaPushEnvCfg (step 5), left as None (disabled) in the Minimal variant --
    success: DoneTerm | None = DoneTerm(
        func=mdp.cube_reached_target, params={"command_name": "target_pos", "threshold": 0.03}
    )
    cube_fell: DoneTerm | None = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("cube")}
    )


##
# Environment configuration
##


@configclass
class PushEnvCfg(ManagerBasedRLEnvCfg):
    """Base config shared by the Minimal and full variants."""

    scene: PushSceneCfg = PushSceneCfg(num_envs=1024, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 2
        self.sim.render_interval = self.decimation
        self.episode_length_s = 6.0
        self.viewer.eye = (2.5, 2.5, 2.5)
        self.sim.dt = 1.0 / 60.0


@configclass
class FrankaPushEnvCfg(PushEnvCfg):
    """Full design (module practicum step 5): reaching shaping, success/failure terminations,
    randomized cube start + target region."""

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # widen the target region -- randomized between episodes
        self.commands.target_pos.ranges.pos_x = (0.5, 0.7)
        self.commands.target_pos.ranges.pos_y = (-0.2, 0.2)

        # randomize the cube's start position between episodes
        self.events.reset_cube.params["pose_range"] = {"x": (-0.1, 0.1), "y": (-0.15, 0.15)}


@configclass
class FrankaPushEnvCfg_Minimal(FrankaPushEnvCfg):
    """First-pass design (module practicum steps 3-4): single reward term, time_out-only
    termination, fixed cube start, fixed single-point target."""

    def __post_init__(self):
        super().__post_init__()

        # fixed single-point target instead of a randomized region
        self.commands.target_pos.ranges.pos_x = (0.6, 0.6)
        self.commands.target_pos.ranges.pos_y = (0.0, 0.0)

        # deterministic cube reset (no randomization yet)
        self.events.reset_cube.params["pose_range"] = {}

        # disable everything except the single distance reward term
        self.rewards.cube_to_target_fine = None
        self.rewards.ee_to_cube = None
        self.rewards.action_rate = None
        self.rewards.joint_vel = None

        # disable everything except time_out
        self.terminations.success = None
        self.terminations.cube_fell = None


@configclass
class FrankaPushEnvCfg_PLAY(FrankaPushEnvCfg):
    """Small-scene variant for rollout / video recording."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
