from openai import OpenAI

client = OpenAI()

with open("LLM/main_prompt.md", "r") as file:
    prompt = file.read()

# user_input = input("Input: ")

# print("Generating Grounded-SAM-2 text prompt...")

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": prompt},
    ],
)

print(completion.choices[0].message.content)
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
