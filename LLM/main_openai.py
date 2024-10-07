from openai import OpenAI
import os

client = OpenAI()

with open("LLM/main_prompt.md", "r") as file:
    prompt = file.read()

with open("temp/user_input.txt", "r") as file:
    user_input = file.read()
    prompt += "\nTask: " + user_input + "\nObject poses:\n"

i = 1
while True:
    object_file = f"temp/objects/object_{i}.txt"
    pose_file = f"temp/objects/pose_world_{i}.txt"

    # Check if the object file exists
    if os.path.isfile(object_file):
        with open(object_file, "r") as file:
            prompt += "\n- " + file.read().rstrip(".") + ": "
    else:
        break

    # Check if the pose file exists
    if os.path.isfile(pose_file):
        with open(pose_file, "r") as file:
            prompt += "`[" + file.readline().strip() + "]`"
    else:
        print(f"Error: Pose file {pose_file} not found")

    i += 1

prompt += "\n"

with open("temp/complete_main_prompt.md", "w") as file:
    file.write(prompt)

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": prompt},
    ],
)

with open("temp/planning_code_snippet.py", "w") as file:
    for line in completion.choices[0].message.content.splitlines():
        if not line.startswith("```"):
            file.write(line + "\n")

file_names = [
    "planning/pick_place_1.py",
    "temp/planning_code_snippet.py",
    "planning/pick_place_3.py",
]
output_file = "temp/planning_full.py"

with open(output_file, "w") as outfile:
    for fname in file_names:
        with open(fname) as infile:
            outfile.write(infile.read())
            outfile.write("\n")
