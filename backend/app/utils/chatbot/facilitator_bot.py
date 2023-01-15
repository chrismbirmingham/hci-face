from facilitator_logic import StatementClassification, DirectorFacilitator, RoleModelFacilitator
from chatgpt import ChatGPT
from zero_shot import ChatLLM, ClassifyLLM


class FacilitatorChat():
    def __init__(self, backend="gpt", facilitor_style="role model") -> None:
        self.facilitator_prompt = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."
        self.sc = StatementClassification()
        self.facilitator_style = facilitor_style
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
        

    def get_bot_response(self, input):
        if self.backend == "gpt":
            self.sc.classify_gpt(self.bot, input)
        if self.backend == "llm":
            self.sc.classify_llm(self.classifier, input)
        tree_response = self.facilitator.decision_tree(self.sc)
        bot_response = self.bot.get_bot_response(input)
        print(tree_response, "\nOR\n", bot_response)
        return tree_response, bot_response

test_input = "That is a great idea. You should try it!"

if __name__ == "__main__":
    bot = FacilitatorChat()
    print(bot.facilitator_prompt)
    print("What would you like to start your conversation with?")
    while True:
        i = input("Human: ")
        response = bot.get_bot_response(i)
        print(response)
