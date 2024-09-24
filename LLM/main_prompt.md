# Background

You will be writing a robotic arm path planning code running in Issac Sim. The code is divided into three parts. The first and the third part are given by me. Your job is to finish the second part of the code so that the three code snippets combine into a complete Python script.

I will give you poses of some objects, which is now only their Cartesian coordinate. Then I will show you your task. Your code should follow the instruction and do the job, which is typically moving a few objects from their current positions to target positions. Do be aware that if you are piling up some objects, you need to first move the object which will eventually be at the bottom of the pile, then the second object counting from bottom-up, etc. Also note that you may need to add a small offset in the z-axis of the target position for objects which are put on other objects. For your information, the first part and third part of the code will be given. At last, for your reference, I will give you some input-output pairs which include object poses, user task and your desired output.

You should only output the second part of the code, which means you don't need to include any code which is already given in the first and the third part. They are just for your information. For my convenience, please only output the code in pure text format, which means you don't need to output any explanation of the code. Don't include any unknown information in your code. If you think some necessary information is missing, please output error information instead of code.

Note that all objects are treated as cubes in simulation for simplicity. For now, assume all cubes have the size of `[0.033, 0.033, 0.038]`.

# Information

## Given Code Snippets

The first part of the code:

```python
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import numpy as np
from controllers.pick_place import PickPlaceController
from omni.isaac.core import World
from tasks.pick_place import PickPlace
import time, os

parser = argparse.ArgumentParser()
parser.add_argument(
    "--test", default=False, action="store_true", help="Run in test mode"
)
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
```

The third part of the code:

```python
my_world.add_task(my_task)
my_world.reset()

my_denso = my_world.scene.get_object("rizon4_robot")
my_controller = PickPlaceController(
    name="controller", robot_articulation=my_denso, gripper=my_denso.gripper
)
task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()

i = 0
action = -1
start_time = time.time()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()

        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]][
                "position"
            ],
            placing_position=observations[task_params["cube_name"]["value"]][
                "target_position"
            ],
            current_joint_positions=observations[task_params["robot_name"]["value"]][
                "joint_positions"
            ],
            end_effector_offset=np.array([0.0, 0, 0.18]),
        )

        if action < my_controller._event:
            end_time = time.time()
            print(
                "action ", str(action), " time:", (end_time - start_time) * 1000, " ms"
            )
            action = my_controller._event
            start_time = time.time()
        num = my_task.task_num
        text = (
            "./standalone_examples/api/omni.isaac.manipulators/RIZON4/simulate_datasets/order"
            + str(num)
            + ".txt"
        )

        os.makedirs(os.path.dirname(text), exist_ok=True)
        with open(text, "a", encoding="utf-8") as file:
            local_action = str(actions)
            local_action = local_action.replace("'", '"')
            file.write(local_action + "\n")

        if my_controller.is_done():
            print("done picking and placing")
            my_controller.reset()
            my_task.task_num += 1
            if my_task.task_num >= my_task._obj_num:
                exit()
            print(my_task.task_num)
            task_params = my_world.get_task("denso_pick_place").get_params()
            action = -1

        articulation_controller.apply_action(actions)

    if args.test is True:
        break

simulation_app.close()
```

## Input-output Example Pairs

### Example 1

Task: Put the apple in the basket.

Object poses:

- apple: `[0.5, 0, 0.2]`
- basket: `[0.4, -0.25, 0.02]`

Desired output:

```python
target_position = [
    np.array([0.4, -0.25, 0.02]),
]
cube_position = [
    np.array([0.5, 0, 0.2]),
]
cube_size = [
    np.array([0.033, 0.033, 0.038]),
]
my_task = PickPlace(
    name="denso_pick_place",
    kind="cube",
    obj_num=1,
    cube_initial_position=cube_position,
    target_position=target_position,
    cube_initial_orientation=None,
    cube_size=cube_size,
)
```

### Example 2

Task: Put the apple in the basket.

Object poses:

- apple: `[0.5, 0, 0.2]`

Desired output:

Error: The pose of the basket is missing.

# Your Task

Task: Put all the drink bottles in the trash bin and put all the fruits in the basket.

Object poses:

- coca-cola: `[0.1, 0.1, 0.1]`
- sprite: `[0.2, 0.3, 0.1]`
- apple: `[-0.1, -0.1, 0.1]`
- pear: `[-0.1, 0, 0.1]`
- trash bin: `[0.3, 0.3, 0.05]`
- basket: `[-0.3, -0.3, 0.05]`