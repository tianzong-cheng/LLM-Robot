import json
import math

output = ""
file_count = 1
line_count = 0
last_joint_positions = [0, 0, 0, 0, 0, 0, 0]


def WrapFile():
    with open(f"temp/order/joint_positions_{file_count}.txt", "r") as file:
        waypoints = file.read()

    with open(f"temp/order/joint_positions_{file_count}.txt", "w") as file:
        file.write(
            "MoveJ(target="
            + output
            + " WORLD WORLD_ORIGIN, "
            + waypoints
            + ", maxVel=0.2)"
        )


if __name__ == "__main__":

    with open("temp/order/joint_positions_1.txt", "w") as file:
        file.write("waypoints=")

    file_path = "temp/order0.txt"
    last_flag = False

    with open(file_path, "r") as file:
        for line in file:
            json_line = line.replace("None", "null")
            data = json.loads(json_line)
            joint_positions = data.get("joint_positions", [])
            flag_not_none = True
            flag_diff = False
            for rad in joint_positions:
                if rad is None:
                    flag_not_none = False

            # Check if joint_positions are different from last_joint_positions
            if flag_not_none:
                for i in range(len(joint_positions)):
                    # abs
                    if (
                        abs(joint_positions[i] - last_joint_positions[i])
                        > 0 / 180 * 3.14
                    ):
                        flag_diff = True
                        break
            if flag_not_none and flag_diff:
                line_count += 1
                last_joint_positions = joint_positions
                joint_positions_degrees = [math.degrees(rad) for rad in joint_positions]
                output = " ".join(f"{deg:.2f}" for deg in joint_positions_degrees)
                with open(f"temp/order/joint_positions_{file_count}.txt", "a") as file:
                    file.write(output + " WORLD WORLD_ORIGIN ")
            if (last_flag and not flag_not_none) or line_count > 70:
                WrapFile()
                file_count += 1
                line_count = 0
                with open(f"temp/order/joint_positions_{file_count}.txt", "w") as file:
                    file.write("waypoints=")
            last_flag = flag_not_none
    WrapFile()
