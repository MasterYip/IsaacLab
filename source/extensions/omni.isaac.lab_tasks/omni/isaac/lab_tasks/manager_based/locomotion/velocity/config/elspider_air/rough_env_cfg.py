# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab_assets.elspider_air import ELSPIDER_AIR_CFG  # isort: skip
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
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
        # self.terminations.base_contact = None
        self.terminations.base_contact = DoneTerm(
            func=mdp.root_height_below_minimum,
            params={"minimum_height": 0.10})


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
