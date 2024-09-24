# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
from typing import Optional

import numpy as np
import omni.isaac.core.tasks as tasks
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper


# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "denso_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        assets_root_path = "/home/tianzong/.local/share/ov/pkg/isaac-sim-2023.1.1/extscache/omni.importer.urdf-1.6.1+105.1.lx64.r.cp310/data"  # get_assets_root_path()
        if assets_root_path is None:
            raise Exception("Could not find Isaac Sim assets folder")

        asset_path = (
            assets_root_path
            + "/urdf/robots/RIZON4/flexiv_rizon4_total/flexiv_rizon4_total.usd"
        )  # "/Isaac/Robots/Flexiv/Rizon4/flexiv_rizon4.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/rizon4")
        gripper = ParallelGripper(
            # We chose the following values while inspecting the articulation
            end_effector_prim_path="/World/rizon4/base_hand",
            joint_prim_names=["left_outer_knuckle_joint", "right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0.5, 0.5]),
            joint_closed_positions=np.array([0, 0]),
            action_deltas=np.array([-0.628, 0.628]),
        )
        manipulator = SingleManipulator(
            prim_path="/World/rizon4",
            name="rizon4_robot",
            end_effector_prim_name="base_hand",
            gripper=gripper,
        )
        joints_default_positions = np.zeros(14)
        joints_default_positions[1] = -0.698
        joints_default_positions[3] = 1.57
        joints_default_positions[5] = 0.698
        joints_default_positions[9] = 0.5
        joints_default_positions[11] = 0.5

        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator
