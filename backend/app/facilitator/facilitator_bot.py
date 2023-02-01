#!/usr/bin/env python3
"""The facilitator bot contains the chatbot functionality for responding as a facilitator.

The faciliator chatbot can either return a generated text response or a logical response
based on difference classification cases of the input.

example: Typical usage (module)
    ```py linenums="1"
    fc = FacilitatorChat()
    statement = input("Type your user input here")
    classes = fc.get_classifications(statement)
    facilitator_response = fc.get_facilitator_response(statement)
    bot_response = fc.get_bot_response(statement)
    ```

note: Independent usage
    For testing or interacting directly the module can also be used as a script:
    ``` bash
    python -m app.facilitator.facilitator_bot
    ```
"""

from .facilitator_logic import StatementClassification, DirectorFacilitator, RoleModelFacilitator
from ..utils import Responder


class FacilitatorChat():
    """Wraps Responder to interactively converse with a facilitator

        Support interaction directly with a prompted openAI model
        or interactin with the custom role model or director models

        Author's Note:
            This is primarily for use in the Support Group Facilitator Study.

        Args:
            chat_backend (str, optional): Which generative model to use. Defaults to "gpt".
            classifier_backend (str, optional): Which classifier model to use. Defaults to "llm".

        Attributes:
            facilitator_prompt (str): Prompt used by the generative chatbots
            classification_processor (obj): Object for processing model classifications into a
                usable format.
            rm_facilitator (obj): Logic for the Role Model Facilitator Condition
            d_facilitator (obj): Logic for the Director Facilitator Condition"""

    def __init__(self, chat_backend="gpt", classifier_backend="llm") -> None:
        self.classifier_backend = classifier_backend
        self.chatbot = Responder(
            chat_backend=chat_backend, classifier_backend=classifier_backend)

        self.facilitator_prompt = ("The following is a conversation with an AI assistant that "
                                   "can have meaningful conversations with users. The assistant is helpful, "
                                   "empathic, and friendly. Its objective is to make the user feel better by "
                                   "feeling heard. With each response, the AI assistant prompts the user to "
                                   "continue the conversation naturally.")

        self.classification_processor = StatementClassification()
        self.rm_facilitator = RoleModelFacilitator()
        self.d_facilitator = DirectorFacilitator()

    def get_classifications(self, statement: str) -> str:
        """Passes the bot to the classification processor.

            Different classifiers are processed differently. In order to properly handle
            the different methods for doing classification,

            Args:
                statement (str): text of statement for classification.

            Returns:
                processed_classifications: joined list of classes that have been classified."""

        if self.classifier_backend == "gpt":
            self.classification_processor.classify_gpt(self.chatbot, statement)

        if self.classifier_backend == "llm":
            self.classification_processor.classify_llm(self.chatbot, statement)

        processed_classifications = self.classification_processor.get_classifications()
        return processed_classifications

    def get_facilitator_response(self, director_condition: bool = False) -> str:
        """Gets facilitator response for either Role Model or Director condition
            
            based on the respective facilitator logic

            Args:
                director_condition (bool, optional): True if in the director condition,
                    false if in the in the role model condition. Defaults to False.

            Returns:
                str: Recommended response to come from the facilitator."""
        if director_condition:
            response = self.d_facilitator.decision_tree(
                self.classification_processor)
        else:
            response = self.rm_facilitator.decision_tree(
                self.classification_processor)
        return response

    def get_bot_response(self, statement: str, speaker: str = "Human",
                         reset_conversation: bool = False) -> str:
        """Get a response from the language model
            based on the prompt, statement, and conversation so far

            Args:
                statement (str): Input statement the bot will respond to.
                speaker (str, optional): Name of the speaker. Defaults to "Human".
                reset_conversation (bool, optional): Resets the conversation to the beginning prompt. Defaults to False.

            Returns:
                str: Text of the bot response"""
        bot_response = self.chatbot.get_response(
            statement, speaker=speaker, reset_conversation=reset_conversation)

        return bot_response


def main():
    """Interactively test the FacilitatorChat

        Must be run from the backend dir:
        $ python -m app.facilitator.facilitator_bot

        Will run until killed with ctrl+c"""
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
