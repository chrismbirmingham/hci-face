if __name__ == "__main__":
    from chatbot.facilitator_logic import StatementClassification, DirectorFacilitator, RoleModelFacilitator
    from chatbot.chatgpt import ChatGPT
    from chatbot.zero_shot import ChatLLM, ClassifyLLM
else:
    from .chatbot.facilitator_logic import StatementClassification, DirectorFacilitator, RoleModelFacilitator
    from .chatbot.chatgpt import ChatGPT
    from .chatbot.zero_shot import ChatLLM, ClassifyLLM




class FacilitatorChat():
    def __init__(self, backend="gpt", facilitator_style="role model") -> None:
        self.facilitator_prompt = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."
        self.sc = StatementClassification()
        self.facilitator_style = facilitator_style
        self.backend = backend
        if backend == "gpt":
            self.bot = ChatGPT(prompt=self.facilitator_prompt)

        if backend == "llm":
            self.bot = ChatLLM(prompt=self.facilitator_prompt)
            self.classifier = ClassifyLLM()

        if self.facilitator_style == "role model":
            self.facilitator = RoleModelFacilitator()
        if self.facilitator_style == "director":
            self.facilitator = DirectorFacilitator()
        

    def get_bot_response(self, input, speaker="Human", reset_conversation=False):
        bot_response = self.bot.get_bot_response(input, speaker=speaker, reset_conversation=reset_conversation)

        if self.backend == "gpt":
            self.sc.classify_gpt(self.bot, input)

        if self.backend == "llm":
            self.sc.classify_llm(self.classifier, input)

        tree_response = self.facilitator.decision_tree(self.sc)

        return tree_response, bot_response
        


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

        tree_response, bot_response = bot.get_bot_response(user_input, speaker=speaker)
        print(f"Tree: {tree_response}\nBot: {bot_response}")

        keep = input("keep response? (n/y tree or bot)")
        if "n" in keep:
            bot.bot.conversation.pop(-1)
        if "tree" in keep:
            if bot.backend == "gpt":
                bot.bot.conversation[-1] = "AI: " + tree_response
            if bot.backend == "llm":
                bot.bot.conversation[-1] = ("AI:", tree_response)
