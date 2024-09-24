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

# import omni.isaac.core.tasks as tasks
from tasks.my_pick_place import MyPickPlace
from tasks.my_gripper import MyGripper
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper

# from omni.isaac.manipulators.grippers import MyGripper


class PickPlace(MyPickPlace):
    def __init__(
        self,
        name: str = "denso_pick_place",
        kind: str = "cube",
        obj_num=1,
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        MyPickPlace.__init__(
            self,
            name=name,
            object_kind=kind,
            obj_num=obj_num,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,  # np.array([0.0515, 0.0515, 0.0515]),
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
        )
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/rizon4")
        gripper = MyGripper(
            # We chose the following values while inspecting the articulation
            end_effector_prim_path="/World/rizon4/base_hand",
            joint_prim_names=[
                "left_outer_knuckle_joint",
                "right_outer_knuckle_joint",
                "left_inner_finger_joint",
                "right_inner_finger_joint",
            ],  #
            joint_opened_positions=np.array([0.6, 0.6, -0.6, -0.6]),
            joint_closed_positions=np.array([0.0, 0.0, 0.0, 0.0]),
            action_deltas=np.array([0.02, 0.02, 0.02, 0.02]),
        )
        manipulator = SingleManipulator(
            prim_path="/World/rizon4",
            name="rizon4_robot",
            end_effector_prim_name="base_hand",
            gripper=gripper,
        )
        #
        joints_default_positions = np.zeros(14)
        joints_default_positions[1] = -0.698
        joints_default_positions[3] = 1.57
        joints_default_positions[5] = 0.698
        joints_default_positions[9] = 0.628
        joints_default_positions[11] = 0.628

        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator
