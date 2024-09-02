
import openai
import numpy as np
import os

# Replace 'your-api-key' with your actual OpenAI API key


def get_instruction_from_chatgpt(prompt):
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",  # Specify the model to use
        messages=[
            {"role": "system", "content": "You should answer with instructions with number listed at the front of each instructions, one instruction one line. The following data is given by a specific sequence, the first line is the target position where the objects need to be put. From the second line, first string represents name of the object, the following 3 are the positions of the object, and the last 3 numbers are the orientation of the object."},
            {"role": "user", "content": prompt}
        ]
    )
    instructions = response.choices[0].message['content']
    return instructions

def get_code_from_chatgpt(prompt):
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",  # Specify the model to use
        messages=[
            {"role": "system", "content": "You should answer only with python code strictly without any other comment such as '```python' or '```'. You must strictly follow the example provided and handle one object and one target position at a time."},
            {"role": "user", "content": prompt}
        ]
    )
    code = response.choices[0].message['content']
    return code

def contains_comments(code):
    if '```python' in code or '```' in code:
        return True
    return False

ins_example = """
Here's example of the potential instruction you need to provide, please follow it.
1. Move object A(name) at (position) with (orientation) to the target place.
2. Move object B(name) at (position) with (orientation) to the target place.
"""
example = """
Here's an example of prompting and the needed response to control the process:
example is:
input: Apple is placed at (0.5,0,0.26), banana is placed at (0.5, -0.15, 0), the apple need to be moved to the target position (0.4,-0.25,0), and the banana is needed to move to the other position at (0.5,-0.3,0.038), the orientation of the objects are apple (0.03, 0.03, 0.03), banana(0.05,0.05,0.05),the size of the apple and banana are (0.33, 0.33, 0.38)
output:
target_position = [np.array([0.4,-0.25,0]),np.array([0.5,-0.3,0.038])]
#change the z value to consider the mass point is not at the bottom.
target_position[0][2] =target_position[0][2]+ 0.042 / 2.0
target_position[1][2] =target_position[1][2]+ 0.042 / 2.0
#assign the cube position
cube_position = [np.array([0.5, 0, 0.26]),np.array([0.5, -0.15, 0])]
cube_position[0][2] = cube_position[0][2]+0.05 / 2.0
cube_position[1][2] = cube_position[1][2]+0.05 / 2.0
cube_orientation = [np.array([0, 0.03, 0.03, 0.03]), np.array([0, 0.05, 0.05, 0.05])]
cube_size=[np.array([0.033, 0.033, 0.038]),np.array([0.033, 0.033, 0.038])]
#assign the number of object 
my_task = PickPlace(name="denso_pick_place",kind="cube",obj_num = 2,cube_initial_position= cube_position,target_position=target_position,cube_initial_orientation  = None,cube_size=cube_size)

Here's another example:
input: Apple is placed at (0.5,0,0.26), banana is placed at (0.5, -0.15, 0), all the objects need to be moved to the target position (0.4,-0.25,0), the orientation of the objects are apple (0.03, 0.03, 0.03), banana(0.05,0.05,0.05),the size of the apple and banana are (0.33, 0.33, 0.38)
output:
cube_size=[np.array([0.033, 0.033, 0.038]),np.array([0.033, 0.033, 0.038])]
target_position = [np.array([0.4,-0.25,0]),np.array([0.4,-0.25,cube_size[0][2]])]
#change the z value to consider the mass point is not at the bottom.
target_position[0][2] =target_position[0][2]+ 0.042 / 2.0
target_position[1][2] =target_position[1][2]+ 0.042 / 2.0
#assign the cube position
cube_position = [np.array([0.5, 0, 0.26]),np.array([0.5, -0.15, 0])]
cube_position[0][2] = cube_position[0][2]+0.05 / 2.0
cube_position[1][2] = cube_position[1][2]+0.05 / 2.0
cube_orientation = [np.array([0, 0.03, 0.03, 0.03]), np.array([0, 0.05, 0.05, 0.05])]

#assign the number of object 
my_task = PickPlace(name="denso_pick_place",kind="cube",obj_num = 2,cube_initial_position= cube_position,target_position=target_position,cube_initial_orientation  = None,cube_size=cube_size)

Based on these example, finish the other task of moving things. You should only modify the numbers in the np.array() in this part. You do not need to change the structure or rename the np, you should just follow the example.
Even they are moviung to the same target position, you should maintain the target position as the same number of the objects. by adding a small offset to 2nd 3rd .. target positions.
The data of objects is given by a specific sequence, the first line is the target position where the objects need to be put. From the second line, first string represents name of the object, the following 3 is the position of the object, and the last 4 numbers is the orientation of the object. The size of the objects just keep it the same as the examples to be 0.033, 0.033, 0.038.
"""

with open('/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/input.txt', 'r') as file:
    object_positions = file.read().strip()  # .strip() to remove any trailing newline or spaces

# Append the extra user prompt to the initial prompt
extra_prompt = input("instruction: ")
prompt_ins = ins_example + " " + object_positions + " " + extra_prompt
generated_ins = get_instruction_from_chatgpt(prompt_ins)

with open('/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/instrutions.txt', 'w') as file:
    file.write(generated_ins)

with open('/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/instrutions.txt', 'r') as file:
    extra_ins = file.read().strip()  # Need to call readline()
    updated_prompt = example + " " + object_positions + " " + extra_ins
    # Generate and check the code 
    while True:
        generated_code = get_code_from_chatgpt(updated_prompt)
        
        with open('/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/part1.txt', 'r') as file_part1:
            part1_code = file_part1.read()
        with open('/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/part2.txt', 'r') as file_part2:
            part2_code = file_part2.read()

        full_code = part1_code + generated_code + part2_code

        if not contains_comments(full_code):
            break
        else:
            print("The generated code contains comments, regenerating...")

    # Print the generated code to inspect it
    print("Generated Code:\n")
    print(generated_code)

    file_name = f'/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/generated_code.py'
    # Write the generated code to a .py file
    with open(file_name, 'w') as output_file:
        output_file.write(part1_code)
        output_file.write(generated_code)
        output_file.write(part2_code)
    
#    extra_ins = file.readline().strip()  # Call readline and .strip() to remove newline

    print(f"The generated code has been written to {file_name}")

# exit()

print(
    "Leave first env"
)
