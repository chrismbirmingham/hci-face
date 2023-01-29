"""Generates responses to text queries"""


from .backends import ChatGPT, ChatLLM, ClassifyLLM


class Responder():
    """Generate chatbot responses"""

    def __init__(self, chat_backend="gpt", classifier_backend="llm") -> None:
        self.default_prompt = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."
        self.chat_backend = chat_backend
        self.classifier_backend = classifier_backend
        
        if chat_backend == "gpt":
            self.bot = ChatGPT(prompt=self.default_prompt)
        if chat_backend == "llm":
            self.bot = ChatLLM(prompt=self.default_prompt)

        if classifier_backend == "gpt":
            self.classifier = ChatGPT()
        if classifier_backend == "llm":
            self.classifier = ClassifyLLM()


    def get_classifications(self, human_input, classes):
        """Gets classification of input statement"""
        if self.classifier_backend == "llm":
            classifications_list = self.classifier.classify(human_input, classes, question="This is an example of {}")
        if self.classifier_backend == "gpt":
            classifications_list = self.classifier.classify(human_input, classes)
        return classifications_list

    def get_response(self, human_input, speaker="Human", reset_conversation=False):
        """Gets bot response to input statement"""
        response = self.bot.get_bot_response(
            human_input, speaker_id=speaker, reset_conversation=reset_conversation)
        return response

    def accept_response(self, response):
        """Keep the bot response in the conversation history"""
        if self.chat_backend == "gpt":
            self.bot.conversation[-1] = "AI: " + response
        if self.chat_backend == "llm":
            self.bot.conversation[-1] = ("AI:", response)

    def reject_response(self):
        """Remove the bot response from the conversation history"""
        self.bot.conversation.pop(-1)

    def get_conversation(self):
        """Returns conversation seen so far"""
        return self.bot.conversation


def main():
    """Test Responder"""

    bot = Responder(chat_backend="gpt", classifier_backend="llm")

    print(bot.default_prompt)
    print("What would you like to start your conversation with?")
    while True:
        prompt_input = input("Speaker: ")

        if prompt_input == "debug":
            print(bot.bot.conversation)
            continue

        user_input = input("Says: ")
        classifications = bot.get_classifications(user_input, ["positive", "negative", "neutral"])
        bot_response = bot.get_response(user_input, prompt_input)
        print(f"Bot: {bot_response} ({classifications})")

        keep = input("keep response? (n/y)")
        if "n" in keep:
            bot.reject_response()
        if "y" in keep:
            bot.accept_response(bot_response)


if __name__ == "__main__":
    main()
