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
