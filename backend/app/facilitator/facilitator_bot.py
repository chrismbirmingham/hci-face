from .facilitator_logic import StatementClassification, DirectorFacilitator, RoleModelFacilitator
# from utils import ChatGPT, ChatLLM, ClassifyLLM
from ..utils import ChatGPT, ChatLLM, ClassifyLLM

# Note, to test this file directly use: python -m facilitator.facilitator_bot from the app directory


class FacilitatorChat():
    def __init__(self, backend="gpt") -> None:
        self.facilitator_prompt = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."
        self.sc = StatementClassification()
        self.backend = backend
        if backend == "gpt":
            self.bot = ChatGPT(prompt=self.facilitator_prompt)

        if backend == "llm":
            self.bot = ChatLLM(prompt=self.facilitator_prompt)
            self.classifier = ClassifyLLM()

        self.rm_facilitator = RoleModelFacilitator()
        self.d_facilitator = DirectorFacilitator()
    
    def get_classifications(self, input):
        if self.backend == "gpt":
            self.sc.classify_gpt(self.bot, input)

        if self.backend == "llm":
            self.sc.classify_llm(self.classifier, input)

        classifications = self.sc.get_classifications()
        return classifications

    def get_facilitator_response(self, director_condition=False):
        if director_condition:
            response = self.d_facilitator.decision_tree(self.sc)
        else:
            response = self.rm_facilitator.decision_tree(self.sc)
        return  response


    def get_bot_response(self, input, speaker="Human", reset_conversation=False):
        bot_response = self.bot.get_bot_response(input, speaker=speaker, reset_conversation=reset_conversation)

        return bot_response
        


if __name__ == "__main__":
    bot = FacilitatorChat(backend="gpt")

    print(bot.facilitator_prompt)
    print("What would you like to start your conversation with?")
    while True:
        speaker = input("Speaker: ")

        if speaker == "debug":
            print(bot.bot.conversation)
            continue

        user_input = input("Says: ")
        classifications = bot.get_classifications(user_input)
        facilitator_response = bot.get_facilitator_response(False)
        bot_response= bot.get_bot_response(user_input, speaker)
        print(f"Tree: {facilitator_response}\nBot: {bot_response}")

        keep = input("keep response? (n/y tree or bot)")
        if "n" in keep:
            bot.bot.conversation.pop(-1)
        if "tree" in keep:
            if bot.backend == "gpt":
                bot.bot.conversation[-1] = "AI: " + facilitator_response
            if bot.backend == "llm":
                bot.bot.conversation[-1] = ("AI:", facilitator_response)