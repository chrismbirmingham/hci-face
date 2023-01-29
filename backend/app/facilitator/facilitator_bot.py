"""Bot for controlling HCI-Face according to the needs of the facilitator"""

from .facilitator_logic import StatementClassification, DirectorFacilitator, RoleModelFacilitator
from ..utils import Responder

# Note, to test this file directly use: python -m facilitator.facilitator_bot from the app directory


class FacilitatorChat():
    """Interactive conversation with a facilitator
    Support interaction directly with a prompted openAI model
    or interactin with the custom role model or director models
    """

    def __init__(self, chat_backend="gpt", classifier_backend="llm") -> None:
        self.facilitator_prompt = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."
        self.classification_processor = StatementClassification()
        self.chatbot = Responder(
            chat_backend=chat_backend, classifier_backend=classifier_backend)
        self.classifier_backend = classifier_backend

        self.rm_facilitator = RoleModelFacilitator()
        self.d_facilitator = DirectorFacilitator()

    def get_classifications(self, statement):
        """Passes the bot to the classification processor

        in order to properly handle the different methods for
        doing classification
        """
        if self.classifier_backend == "gpt":
            self.classification_processor.classify_gpt(self.chatbot, statement)

        if self.classifier_backend == "llm":
            self.classification_processor.classify_llm(self.chatbot, statement)

        processed_classifications = self.classification_processor.get_classifications()
        return processed_classifications

    def get_facilitator_response(self, director_condition=False):
        """Get facilitator response for either Role Model or Director condition
        based on the respective facilitator logic
        """
        if director_condition:
            response = self.d_facilitator.decision_tree(
                self.classification_processor)
        else:
            response = self.rm_facilitator.decision_tree(
                self.classification_processor)
        return response

    def get_bot_response(self, statement, speaker="Human", reset_conversation=False):
        """Get a response from the language model based on the prompt, statement, and conversation so far"""
        bot_response = self.chatbot.get_response(
            statement, speaker=speaker, reset_conversation=reset_conversation)

        return bot_response


def main():
    """Interactively test the FacilitatorChat

    Must be run from the backend dir:
    python -m app.facilitator.facilitator_bot
    """
    bot = FacilitatorChat(chat_backend="gpt", classifier_backend="llm")

    print(bot.facilitator_prompt)
    print("What would you like to start your conversation with?")
    while True:
        identified_speaker = input("Speaker: ")

        if identified_speaker == "debug":
            print(bot.chatbot.get_conversation())
            continue

        user_input = input("Says: ")
        classifications = bot.get_classifications(user_input)
        facilitator_response = bot.get_facilitator_response(False)
        bot_response = bot.get_bot_response(user_input, identified_speaker)
        print(
            f"Tree: {facilitator_response}\nBot: {bot_response}\nClasses: {classifications}")

        keep = input("keep response? (n/y tree or bot)")
        if "n" in keep:
            bot.chatbot.reject_response()
        if "tree" in keep:
            bot.chatbot.accept_response(facilitator_response)
        if "bot" in keep:
            bot.chatbot.accept_response(bot_response)


if __name__ == "__main__":
    main()
