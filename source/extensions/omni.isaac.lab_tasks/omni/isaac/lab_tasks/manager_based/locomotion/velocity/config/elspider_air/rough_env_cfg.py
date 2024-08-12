# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##

import math
from omni.isaac.lab_assets.elspider_air import ELSPIDER_AIR_CFG  # isort: skip
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
import omni.isaac.lab_tasks.manager_based.locomotion.velocity.mdp as mdp


@configclass
class ElSpiderAirRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # switch robot to elspider air
        self.scene.robot = ELSPIDER_AIR_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # FIXME: the base contact_force is triggered randomly
        # self.terminations.base_contact = DoneTerm(
        #     func=mdp.illegal_contact,
        #     params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
        # )

        self.terminations.base_contact = None
        # self.terminations.base_contact = DoneTerm(
        #     func=mdp.root_height_below_minimum,
        #     params={"minimum_height": 0.10})
        self.rewards.track_lin_vel_xy_exp = RewTerm(
            func=mdp.track_lin_vel_xy_exp, weight=5.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
        )
        self.rewards.track_ang_vel_z_exp = RewTerm(
            func=mdp.track_ang_vel_z_exp, weight=2.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
        )
        self.rewards.dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-5e-9)

        self.rewards.undesired_contacts = RewTerm(
            func=mdp.undesired_contacts,
            weight=-5.0,
            params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*(THIGH|HIP)"), "threshold": 1.0},
        )

        self.rewards.feet_air_time_penalty = RewTerm(
            func=mdp.feet_air_time_pena,
            weight=1.0,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*FOOT"),
                "command_name": "base_velocity",
                "threshold": 5,
            },
        )


@ configclass
class ElSpiderAirRoughEnvCfg_PLAY(ElSpiderAirRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
