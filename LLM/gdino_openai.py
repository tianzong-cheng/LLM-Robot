from openai import OpenAI

client = OpenAI()

with open("LLM/gdino_prompt.md", "r") as file:
    prompt = file.read()

user_input = input("Input: ")

print("Generating Grounded-SAM-2 text prompt...")

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": prompt + user_input},
    ],
)

print("Grounded-SAM-2 text prompt: " + completion.choices[0].message.content)
with open("temp/gdino_text_prompt.txt", "w") as file:
    file.write(completion.choices[0].message.content)
