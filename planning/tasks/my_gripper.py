# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Callable, List

import numpy as np
import omni.kit.app
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators.grippers.gripper import Gripper
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper


class MyGripper(ParallelGripper):
    """Provides high level functions to set/ get properties and actions of a parllel gripper
    (a gripper that has two fingers).

    Args:
        end_effector_prim_path (str): prim path of the Prim that corresponds to the gripper root/ end effector.
        joint_prim_names (List[str]): the left finger joint prim name and the right finger joint prim name respectively.
        joint_opened_positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively when opened.
        joint_closed_positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively when closed.
        action_deltas (np.ndarray, optional): deltas to apply for finger joint positions when openning or closing the gripper. Defaults to None.
    """

    def __init__(
        self,
        end_effector_prim_path: str,
        joint_prim_names: List[str],
        joint_opened_positions: np.ndarray,
        joint_closed_positions: np.ndarray,
        action_deltas: np.ndarray = None,
    ) -> None:
        Gripper.__init__(self, end_effector_prim_path=end_effector_prim_path)
        self._joint_prim_names = joint_prim_names
        self._joint_dof_indicies = np.array([None, None, None, None])
        self._joint_opened_positions = joint_opened_positions
        self._joint_closed_positions = joint_closed_positions
        self._get_joint_positions_func = None
        self._set_joint_positions_func = None
        self._action_deltas = action_deltas
        self._articulation_num_dofs = None
        return

    @property
    def joint_opened_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively when opened.
        """
        return self._joint_opened_positions

    @property
    def joint_closed_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively when closed.
        """
        return self._joint_closed_positions

    @property
    def joint_dof_indicies(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint dof indices in the articulation of the left finger joint and the right finger joint respectively.
        """
        return self._joint_dof_indicies

    @property
    def joint_prim_names(self) -> List[str]:
        """
        Returns:
            List[str]: the left finger joint prim name and the right finger joint prim name respectively.
        """
        return self._joint_prim_names

    def initialize(
        self,
        articulation_apply_action_func: Callable,
        get_joint_positions_func: Callable,
        set_joint_positions_func: Callable,
        dof_names: List,
        physics_sim_view: omni.physics.tensors.SimulationView = None,
    ) -> None:
        """Create a physics simulation view if not passed and creates a rigid prim view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            articulation_apply_action_func (Callable): apply_action function from the Articulation class.
            get_joint_positions_func (Callable): get_joint_positions function from the Articulation class.
            set_joint_positions_func (Callable): set_joint_positions function from the Articulation class.
            dof_names (List): dof names from the Articulation class.
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None

        Raises:
            Exception: _description_
        """
        print("initialize is run")
        Gripper.initialize(self, physics_sim_view=physics_sim_view)
        self._get_joint_positions_func = get_joint_positions_func
        self._articulation_num_dofs = len(dof_names)
        for index in range(len(dof_names)):
            if self._joint_prim_names[0] == dof_names[index]:
                self._joint_dof_indicies[0] = index
            elif self._joint_prim_names[1] == dof_names[index]:
                self._joint_dof_indicies[1] = index
            elif self._joint_prim_names[2] == dof_names[index]:
                self._joint_dof_indicies[2] = index
            elif self._joint_prim_names[3] == dof_names[index]:
                self._joint_dof_indicies[3] = index
        # make sure that all gripper dof names were resolved
        if self._joint_dof_indicies[0] is None or self._joint_dof_indicies[1] is None:
            raise Exception(
                "Not all gripper dof names were resolved to dof handles and dof indices."
            )
        self._articulation_apply_action_func = articulation_apply_action_func
        current_joint_positions = get_joint_positions_func()
        if self._default_state is None:
            self._default_state = np.array(
                [
                    current_joint_positions[self._joint_dof_indicies[0]],
                    current_joint_positions[self._joint_dof_indicies[1]],
                    current_joint_positions[self._joint_dof_indicies[2]],
                    current_joint_positions[self._joint_dof_indicies[3]],
                ]
            )
        self._set_joint_positions_func = set_joint_positions_func
        return

    def open(self) -> None:
        """Applies actions to the articulation that opens the gripper (ex: to release an object held)."""
        self._articulation_apply_action_func(self.forward(action="open"))
        return

    def close(self) -> None:
        """Applies actions to the articulation that closes the gripper (ex: to hold an object)."""
        self._articulation_apply_action_func(self.forward(action="close"))
        return

    def set_action_deltas(self, value: np.ndarray) -> None:
        """
        Args:
            value (np.ndarray): deltas to apply for finger joint positions when openning or closing the gripper.
                               [left, right]. Defaults to None.
        """
        self._action_deltas = value
        return

    def get_action_deltas(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: deltas that will be applied for finger joint positions when openning or closing the gripper.
                        [left, right]. Defaults to None.
        """
        return self._action_deltas

    def set_default_state(self, joint_positions: np.ndarray) -> None:
        """Sets the default state of the gripper

        Args:
            joint_positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively.
        """
        self._default_state = joint_positions
        return

    def get_default_state(self) -> np.ndarray:
        """Gets the default state of the gripper

        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._default_state

    def post_reset(self):
        Gripper.post_reset(self)
        self._set_joint_positions_func(
            positions=self._default_state,
            joint_indices=[
                self._joint_dof_indicies[0],
                self._joint_dof_indicies[1],
                self._joint_dof_indicies[2],
                self._joint_dof_indicies[3],
            ],
        )
        return

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """
        Args:
            positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively.
        """
        self._set_joint_positions_func(
            positions=positions,
            joint_indices=[
                self._joint_dof_indicies[0],
                self._joint_dof_indicies[1],
                self._joint_dof_indicies[2],
                self._joint_dof_indicies[3],
            ],
        )
        return

    def get_joint_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._get_joint_positions_func(
            joint_indices=[
                self._joint_dof_indicies[0],
                self._joint_dof_indicies[1],
                self._joint_dof_indicies[2],
                self._joint_dof_indicies[3],
            ]
        )

    def forward(self, action: str) -> ArticulationAction:
        """calculates the ArticulationAction for all of the articulation joints that corresponds to "open"
           or "close" actions.

        Args:
            action (str): "open" or "close" as an abstract action.

        Raises:
            Exception: _description_

        Returns:
            ArticulationAction: articulation action to be passed to the articulation itself
                                (includes all joints of the articulation).
        """
        if action == "open":
            target_joint_positions = [None] * self._articulation_num_dofs
            if self._action_deltas is None:
                target_joint_positions[self._joint_dof_indicies[0]] = (
                    self._joint_opened_positions[0]
                )
                target_joint_positions[self._joint_dof_indicies[1]] = (
                    self._joint_opened_positions[1]
                )
                target_joint_positions[self._joint_dof_indicies[2]] = (
                    self._joint_opened_positions[2]
                )
                target_joint_positions[self._joint_dof_indicies[3]] = (
                    self._joint_opened_positions[3]
                )
            else:
                current_joint_positions = self._get_joint_positions_func()
                current_left_finger_position = current_joint_positions[
                    self._joint_dof_indicies[0]
                ]
                current_right_finger_position = current_joint_positions[
                    self._joint_dof_indicies[1]
                ]
                current_left_inner_finger_position = current_joint_positions[
                    self._joint_dof_indicies[2]
                ]
                current_right_inner_finger_position = current_joint_positions[
                    self._joint_dof_indicies[3]
                ]
                target_joint_positions[self._joint_dof_indicies[0]] = (
                    current_right_finger_position + self._action_deltas[0]
                )
                # (
                #    min(current_left_finger_position + self._action_deltas[0],self._joint_opened_positions[0])
                # )
                target_joint_positions[self._joint_dof_indicies[1]] = (
                    current_right_finger_position + self._action_deltas[1]
                )
                # (
                #    min(current_right_finger_position + self._action_deltas[1],self._joint_opened_positions[1])
                # )
                target_joint_positions[self._joint_dof_indicies[2]] = (
                    current_left_inner_finger_position - self._action_deltas[2]
                )
                # (
                #    max(current_left_inner_finger_position - self._action_deltas[2],self._joint_opened_positions[2])
                # )
                target_joint_positions[self._joint_dof_indicies[3]] = (
                    current_right_inner_finger_position - self._action_deltas[3]
                )
                # (
                #    max(current_right_inner_finger_position - self._action_deltas[3],self._joint_opened_positions[3])
                # )
        elif action == "close":
            target_joint_positions = [None] * self._articulation_num_dofs
            if self._action_deltas is None:
                target_joint_positions[self._joint_dof_indicies[0]] = (
                    self._joint_closed_positions[0]
                )
                target_joint_positions[self._joint_dof_indicies[1]] = (
                    self._joint_closed_positions[1]
                )
                target_joint_positions[self._joint_dof_indicies[2]] = (
                    self._joint_closed_positions[2]
                )
                target_joint_positions[self._joint_dof_indicies[3]] = (
                    self._joint_closed_positions[3]
                )
            else:
                current_joint_positions = self._get_joint_positions_func()
                current_left_finger_position = current_joint_positions[
                    self._joint_dof_indicies[0]
                ]
                current_right_finger_position = current_joint_positions[
                    self._joint_dof_indicies[1]
                ]
                current_left_inner_finger_position = current_joint_positions[
                    self._joint_dof_indicies[2]
                ]
                current_right_inner_finger_position = current_joint_positions[
                    self._joint_dof_indicies[3]
                ]
                target_joint_positions[self._joint_dof_indicies[0]] = (
                    current_left_finger_position - self._action_deltas[0]
                )
                target_joint_positions[self._joint_dof_indicies[1]] = (
                    current_left_finger_position - self._action_deltas[1]
                )
                target_joint_positions[self._joint_dof_indicies[2]] = (
                    current_left_inner_finger_position + self._action_deltas[2]
                )

                target_joint_positions[self._joint_dof_indicies[3]] = (
                    current_right_inner_finger_position + self._action_deltas[3]
                )

            print(self._action_deltas)

            # print(current_joint_positions)
            print(self._joint_closed_positions)
        else:
            raise Exception(
                "action {} is not defined for ParallelGripper".format(action)
            )
        return ArticulationAction(joint_positions=target_joint_positions)

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """Applies actions to all the joints of an articulation that corresponds to the ArticulationAction of the finger joints only.

        Args:
            control_actions (ArticulationAction): ArticulationAction for the left finger joint and the right finger joint respectively.
        """
        joint_actions = ArticulationAction()
        if control_actions.joint_positions is not None:
            joint_actions.joint_positions = [None] * self._articulation_num_dofs
            joint_actions.joint_positions[self._joint_dof_indicies[0]] = (
                control_actions.joint_positions[0]
            )
            joint_actions.joint_positions[self._joint_dof_indicies[1]] = (
                control_actions.joint_positions[1]
            )
            joint_actions.joint_positions[self._joint_dof_indicies[2]] = (
                control_actions.joint_positions[2]
            )
            joint_actions.joint_positions[self._joint_dof_indicies[3]] = (
                control_actions.joint_positions[3]
            )
        if control_actions.joint_velocities is not None:
            joint_actions.joint_velocities = [None] * self._articulation_num_dofs
            joint_actions.joint_velocities[self._joint_dof_indicies[0]] = (
                control_actions.joint_velocities[0]
            )
            joint_actions.joint_velocities[self._joint_dof_indicies[1]] = (
                control_actions.joint_velocities[1]
            )
            joint_actions.joint_velocities[self._joint_dof_indicies[0]] = (
                control_actions.joint_velocities[2]
            )
            joint_actions.joint_velocities[self._joint_dof_indicies[1]] = (
                control_actions.joint_velocities[3]
            )
        if control_actions.joint_efforts is not None:
            joint_actions.joint_efforts = [None] * self._articulation_num_dofs
            joint_actions.joint_efforts[self._joint_dof_indicies[0]] = (
                control_actions.joint_efforts[0]
            )
            joint_actions.joint_efforts[self._joint_dof_indicies[1]] = (
                control_actions.joint_efforts[1]
            )
            joint_actions.joint_efforts[self._joint_dof_indicies[0]] = (
                control_actions.joint_efforts[2]
            )
            joint_actions.joint_efforts[self._joint_dof_indicies[1]] = (
                control_actions.joint_efforts[3]
            )
        self._articulation_apply_action_func(control_actions=joint_actions)
        return
