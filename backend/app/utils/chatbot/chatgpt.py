import openai
import os

openai.api_key = os.getenv("OPENAI_API_KEY")
openai.Model.list()


default_prompt = "The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n"

class ChatGPT():
    def __init__(self, prompt=default_prompt) -> None:
        self.prompt = prompt
        self.conversation = [prompt]
        pass

    def get_bot_response(self, user_input, speaker="Human", reset_conversation=False):
        if reset_conversation: 
            self.conversation = [self.prompt]

        self.conversation.append(f"{speaker}: {user_input}")
        self.conversation.append("AI: ")

        input_prompt = "\n".join(self.conversation)

        response = self._queryAPI(input_prompt)["choices"][0]["text"]
        response = response.replace("\n",'')

        self.conversation[-1] += response
        return response

    def _queryAPI(self, prompt, stop=[" Human:", " AI:"], temp=.9, mt=150):
        # Generate a response
        response = openai.Completion.create(
            model="text-davinci-003",
            temperature=temp,
            prompt = prompt,
            max_tokens=mt,
            top_p=1,
            frequency_penalty=0.0,
            presence_penalty=0.6,
            stop=stop
        )
        return response

    def answer_question_on_input(self, input, question):
        prompt = f"{input}\n\n{question}\nAnswer each of these questions as separate sentences.\n\n"
        stop = ["\n\n"]
        response = self._queryAPI(prompt, stop=stop, temp=0, mt=80)
        answer_sentence = response["choices"][0]["text"]
        return answer_sentence


if __name__ == "__main__":
    print(default_prompt)
    print("What would you like to start your conversation with?")
    bot = ChatGPT()
    while True:
        speaker = input("Speaker: ")
        if speaker == "debug":
            print(bot.conversation)
        user_input = input("Says: ")
        response = bot.get_bot_response(user_input, speaker=speaker)
        print(response)
        keep = input("keep response? (y/n)")
        if keep == "n":
            bot.conversation.pop(-1)
