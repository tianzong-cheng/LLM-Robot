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


tcp_pose_matrix = np.array(
    [[-1, 0, 0, 0.6], [0, 1, 0, -0.2], [0, 0, -1, 0.6], [0, 0, 0, 1]]
)

transformation_matrix = np.array(
    [
        [0.01035566, 0.99726347, 0.07320067, -0.06347553],
        [-0.99994025, 0.01058405, -0.00273277, 0.1598579],
        [-0.00350005, -0.073168, 0.99731349, -0.14802994],
        [0, 0, 0, 1],
    ]
)

camera_pose_matrix = tcp_pose_matrix @ transformation_matrix

camera_position = camera_pose_matrix[:3, 3]
camera_quaternion = R.from_matrix(camera_pose_matrix[:3, :3]).as_quat()
camera_world_pose = np.concatenate([camera_position, camera_quaternion])

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
