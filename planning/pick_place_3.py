cube_size = np.array([0.04, 0.04, 0.05])
cube_sizes = [cube_size for _ in range(subtask_num)]
my_task = PickPlace(
    name="denso_pick_place",
    kind="cube",
    obj_num=subtask_num,
    cube_initial_position=initial_position,
    target_position=target_position,
    cube_initial_orientation=None,
    cube_size=cube_sizes,
)

my_world.add_task(my_task)
my_world.reset()

my_denso = my_world.scene.get_object("rizon4_robot")
my_controller = PickPlaceController(
    name="controller", robot_articulation=my_denso, gripper=my_denso.gripper
)
task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()
last_end_effector_position = np.array([0, 0, 0])
i = 0
action = -1
start_time = time.time()

while simulation_app.is_running():
    my_world.step(render=False)
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

        num = my_task.task_num
        text = (
            "./standalone_examples/api/omni.isaac.manipulators/RIZON4/simulate_datasets/order"
            + str(num)
            + ".txt"
        )
        end_effector_position = observations[task_params["robot_name"]["value"]][
            "end_effector_position"
        ]

        if (
            not last_end_effector_position.any()
            or np.linalg.norm(last_end_effector_position - end_effector_position) > 0.06
            or action < my_controller._event
        ):
            with open(text, "a", encoding="utf-8") as file:
                local_action = str(actions)
                local_action = local_action.replace("'", '"')
                file.write(local_action + "\n")
                # file.write(str(my_controller._event) + "\n")
                print(end_effector_position)
                last_end_effector_position = end_effector_position

        if action < my_controller._event:
            end_time = time.time()
            print(
                "action ", str(action), " time:", (end_time - start_time) * 1000, " ms"
            )
            action = my_controller._event
            start_time = time.time()

        if my_controller.is_done():
            print("done picking and placing")
            my_controller.reset()
            my_task.task_num += 1
            if my_task.task_num >= my_task._obj_num:
                exit()
            task_params = my_world.get_task("denso_pick_place").get_params()
            action = -1
            last_joint_position = np.array([0, 0, 0])

        articulation_controller.apply_action(actions)
    if args.test is True:
        break

simulation_app.close()
