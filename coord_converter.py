import numpy as np
import os
from scipy.spatial.transform import Rotation as R


def transform_object_pose(camera_world_pose, object_camera_pose):
    camera_position = np.array(camera_world_pose[:3])
    camera_quaternion = np.array(camera_world_pose[3:])

    rotation = R.from_quat(camera_quaternion).as_matrix()

    object_position_camera = np.array(object_camera_pose)

    object_position_world = camera_position + rotation.dot(object_position_camera)

    return object_position_world


camera_world_pose = [0.6, -0.2, 0.6, 0.0, 1.0, 0.0, 0.0]  # [x, y, z, qx, qy, qz, qw]

i = 1
while True:
    if os.path.isfile(f"temp/objects/pose_camera_{i}.txt"):
        with open(f"temp/objects/pose_camera_{i}.txt", "r") as f:
            object_camera_pose = list(map(float, f.readline().strip().split()))

        object_world_pose = transform_object_pose(camera_world_pose, object_camera_pose)

        with open(f"temp/objects/pose_world_{i}.txt", "w") as f:
            f.write(", ".join(map(str, object_world_pose)))

        i += 1
    else:
        break
