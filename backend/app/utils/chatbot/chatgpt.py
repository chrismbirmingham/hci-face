import openai
import os

openai.api_key = os.getenv("OPENAI_API_KEY")
openai.Model.list()


conversation_prompt = "The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n"

class ChatGPT():
    def __init__(self, prompt=conversation_prompt) -> None:
        self.prompt = prompt
        self.conversation = [prompt]
        pass

    def get_bot_response(self, user_input, reset_conversation=False):
        if reset_conversation: self.conversation = [self.prompt]
        self.conversation.append("Human: " + user_input)
        self.conversation.append("AI: ")
        prompt = "\n".join(self.conversation)
        # print(prompt)
        response = self.queryAPI(prompt)["choices"][0]["text"]
        response = response.replace("\n",'')
        self.conversation.append(response)
        return response

    def queryAPI(self, prompt):
        # Generate a response
        response = openai.Completion.create(
            model="text-davinci-003",
            temperature=0.9,
            prompt = prompt,
            max_tokens=150,
            top_p=1,
            frequency_penalty=0.0,
            presence_penalty=0.6,
            stop=[" Human:", " AI:"]
        )
        return response



if __name__ == "__main__":
    print(conversation_prompt)
    print("What would you like to start your conversation with?")
    cgpt = ChatGPT()
    while True:
        i = input("Human: ")
        response = cgpt.get_bot_response(i)
        print(response)