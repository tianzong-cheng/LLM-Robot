from openai import OpenAI

client = OpenAI()

with open("gdino_prompt.md", "r") as file:
    prompt = file.read()

user_input = input("Input: ")

print("Generating response...")

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": prompt + user_input},
    ],
)

print(completion.choices[0].message.content)
