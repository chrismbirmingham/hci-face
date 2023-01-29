"""Wrapper for communicating with openAI's davinci 3 model"""


import os
import openai

openai.api_key = os.getenv("OPENAI_API_KEY")
openai.Model.list()


DEFAULT = "The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n"


class ChatGPT:
    """Wraps the functionality for using the OpenAI api to get responses

    Maintains a record of the conversation which is fed to the model.
    """

    def __init__(self, prompt: str = DEFAULT, model: str = "text-davinci-003") -> None:
        self.backend = "gpt"
        self.model = model
        self.prompt = prompt
        self.conversation = [prompt]

    def query_API(self, prompt, stop=[" Human:", " AI:"], temp=.9, max_tokens=150):
        """Generate a response"""

        api_response = openai.Completion.create(
            model=self.model,
            temperature=temp,
            prompt=prompt,
            max_tokens=max_tokens,
            top_p=1,
            frequency_penalty=0.0,
            presence_penalty=0.6,
            stop=stop
        )
        return api_response

    def get_bot_response(self, statement, speaker_id="Human", reset_conversation=False):
        """Calls the API with user input and conversation history to get bot response"""
        if reset_conversation:
            self.conversation = [self.prompt]

        self.conversation.append(f"{speaker_id}: {statement}")
        self.conversation.append("AI: ")

        input_prompt = "\n".join(self.conversation)

        api_response = self.query_API(input_prompt)["choices"][0]["text"]
        api_response = api_response.replace("\n", '')

        self.conversation[-1] += api_response
        return api_response

    def classify(self, statement, classes, question="Should the prior statement be classified as"):
        """Classify with an input question"""
        classes_str = ", ".join(classes[:-1]) + f", or {classes[-1]}?"
        prompt = f"{statement}\n\n{question} {classes_str}\n\n"
        stop = ["\n\n"]

        api_response = self.query_API(prompt, stop=stop, temp=0, max_tokens=80)
        answer_sentence = api_response["choices"][0]["text"]

        return answer_sentence

    def answer_questions(self, statement, questions):
        """Answer a set of questions, it helps if they are numbered"""
        prompt = f"{statement}\n\nPlease answer the following questions individually:\n{questions}\n\n"
        stop = ["\n\n"]

        api_response = self.query_API(prompt, stop=stop, temp=0, max_tokens=80)
        answer_sentence = api_response["choices"][0]["text"]

        return answer_sentence


if __name__ == "__main__":
    print(DEFAULT)
    print("What would you like to start your conversation with?")
    bot = ChatGPT()
    while True:
        speaker = input("Speaker: ")
        if speaker == "debug":
            print(bot.conversation)

        user_input = input("Says: ")

        response = bot.get_bot_response(user_input, speaker_id=speaker)
        valence = bot.classify(user_input, ["positive", "negative", "neutral"])
        print(response, valence)
        keep = input("keep response? (y/n)")
        if keep == "n":
            bot.conversation.pop(-1)
