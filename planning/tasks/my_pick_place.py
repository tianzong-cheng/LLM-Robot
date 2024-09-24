# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects import DynamicCapsule
from omni.isaac.core.objects import DynamicCylinder
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.string import find_unique_string_name
from typing import List


class MyPickPlace(ABC, BaseTask):
    """[summary]

    Args:
        name (str): [description]
        cube_initial_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_initial_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str,
        object_kind: str,
        obj_num: int,
        cube_initial_position: Optional[List[np.ndarray]] = None,
        cube_initial_orientation: Optional[List[np.ndarray]] = None,
        target_position: Optional[List[np.ndarray]] = None,
        cube_size: Optional[List[np.ndarray]] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._robot = None
        self._target_cube = None
        self._kind = object_kind
        self._obj_num = obj_num
        self._objs = []
        self._object_initial_position = cube_initial_position
        self._object_initial_orientation = cube_initial_orientation
        self._target_position = target_position
        self._cube_size = cube_size
        self.task_num = 0

        if self._cube_size is None:
            self._cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
        if self._object_initial_position is None:
            self._cube_initial_position = np.array([0.3, 0.3, 0.3]) / get_stage_units()
        if self._object_initial_orientation is None:
            self._object_initial_orientation = [np.array([1, 0, 0, 0])] * obj_num
        if self._target_position is None:
            self._target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
            self._target_position[2] = self._cube_size[2] / 2.0

        self._target_position = self._target_position + self._offset
        return

    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        for i in range(self._obj_num):
            cube_prim_path = find_unique_string_name(
                initial_name="/World/Cube" + str(i),
                is_unique_fn=lambda x: not is_prim_path_valid(x),
            )
            cube_name = find_unique_string_name(
                initial_name=self._kind,
                is_unique_fn=lambda x: not self.scene.object_exists(x),
            )
            if self._kind == "cube":
                _obj = DynamicCuboid(
                    name=cube_name,
                    position=self._object_initial_position[i],
                    orientation=self._object_initial_orientation[i],
                    prim_path=cube_prim_path,
                    scale=self._cube_size[i],
                    size=1.0,
                    color=np.array([0, 0, 1]),
                )
            elif self._kind == "sphere":
                _obj = DynamicSphere(
                    name=cube_name,
                    position=self._object_initial_position[i],
                    orientation=self._object_initial_orientation[i],
                    prim_path=cube_prim_path,
                    scale=self._cube_size[i],
                    color=np.array([0, 0, 1]),
                )
            elif self._kind == "cylinder":
                _obj = DynamicCylinder(
                    name=cube_name,
                    position=self._object_initial_position[i],
                    orientation=self._object_initial_orientation[i],
                    prim_path=cube_prim_path,
                    scale=self._cube_size[i],
                    color=np.array([0, 0, 1]),
                )
            elif self._kind == "capsule":
                _obj = DynamicCapsule(
                    name=cube_name,
                    position=self._object_initial_position[i],
                    orientation=self._object_initial_orientation[i],
                    prim_path=cube_prim_path,
                    scale=self._cube_size[i],
                    color=np.array([0, 0, 1]),
                )
            # self._object = scene.add(self._obj)
            self._task_objects[_obj.name] = _obj
            self._objs.append(_obj)

            # place a cube under _obj if z is larger than 0
            if self._object_initial_position[i][2] > self._cube_size[i][2] / 2 + 0.01:
                print("[INFO]")
                print(
                    f"_object_initial_position[{i}]: {self._object_initial_position[i]}"
                )
                print(f"_cube_size[{i}]: {self._cube_size[i]}")
                cube_prim_path = find_unique_string_name(
                    initial_name="/World/Cube" + str(i),
                    is_unique_fn=lambda x: not is_prim_path_valid(x),
                )
                cube_name = find_unique_string_name(
                    initial_name="cube",
                    is_unique_fn=lambda x: not self.scene.object_exists(x),
                )
                new_position = self._object_initial_position[i]
                new_position[2] = (new_position[2] - self._cube_size[i][2] / 2) / 2
                new_scale = np.array([0.1, 0.1, new_position[2] * 2])
                print(f"new_position: {new_position}")
                print(f"new_scale: {new_scale}")
                _obj = FixedCuboid(
                    name=cube_name,
                    position=new_position,
                    orientation=self._object_initial_orientation[i],
                    prim_path=cube_prim_path,
                    scale=new_scale,
                    color=np.array([0, 1, 0]),
                )
                self._task_objects[_obj.name] = _obj

        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return

    @abstractmethod
    def set_robot(self) -> None:
        raise NotImplementedError

    def set_params(
        self,
        cube_position: Optional[np.ndarray] = None,
        cube_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
    ) -> None:
        if target_position is not None:
            self._target_position = target_position
        if cube_position is not None or cube_orientation is not None:
            self._obj.set_local_pose(
                translation=cube_position, orientation=cube_orientation
            )
        return

    def get_params(self) -> dict:
        params_representation = dict()
        position, orientation = self._objs[self.task_num].get_local_pose()
        target_position = self._target_position[self.task_num]
        params_representation["cube_position"] = {"value": position, "modifiable": True}
        params_representation["cube_orientation"] = {
            "value": orientation,
            "modifiable": True,
        }
        params_representation["target_position"] = {
            "value": target_position,
            "modifiable": True,
        }
        params_representation["cube_name"] = {
            "value": self._objs[self.task_num].name,
            "modifiable": False,
        }
        params_representation["robot_name"] = {
            "value": self._robot.name,
            "modifiable": False,
        }
        return params_representation

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        cube_position, cube_orientation = self._objs[self.task_num].get_local_pose()
        target_position = self._target_position[self.task_num]
        end_effector_position, _ = self._robot.end_effector.get_local_pose()
        return {
            self._objs[self.task_num].name: {
                "position": cube_position,
                "orientation": cube_orientation,
                "target_position": target_position,
            },
            self._robot.name: {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
            },
        }

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """[summary]

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        return

    def post_reset(self) -> None:
        from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

        if isinstance(self._robot.gripper, ParallelGripper):
            self._robot.gripper.set_joint_positions(
                self._robot.gripper.joint_opened_positions
            )
        return

    def calculate_metrics(self) -> dict:
        """[summary]"""
        raise NotImplementedError

    def is_done(self) -> bool:
        """[summary]"""
        raise NotImplementedError
