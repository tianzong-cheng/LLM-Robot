from openai import OpenAI

client = OpenAI()

with open("LLM/gdino_prompt.md", "r") as file:
    prompt = file.read()

with open("temp/user_input.txt", "r") as file:
    user_input = file.read()

print("Generating Grounded-SAM-2 text prompt...")

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": prompt + user_input},
    ],
)

print("Grounded-SAM-2 text prompt: " + completion.choices[0].message.content)

list = completion.choices[0].message.content.split(".")

i = 0

for object in list:
    if object:
        i = i + 1
        file_name = f"object_{i}.txt"
        with open("temp/objects/" + file_name, "w") as file:
            file.write(object.strip() + ".")
