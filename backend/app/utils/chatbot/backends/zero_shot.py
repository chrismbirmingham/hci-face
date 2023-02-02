#!/usr/bin/env python3
"""This module for using models to do zero shot classification and text generation

This implements a wrapper on huggingface generator and classifier pipelines.

warning: Generation models are big and must be stored locally.
    The generative models are expected to be stored in the MODEL_DICT.
    When you use them for the first time they will be downloaded and
    stored in the 'pt_path' please change this to a location that makes
    sense on your local machine.
    
warning: Large models need RAM and GPU support to run efficiently
    Classification may or may not work depending on your model choice, but
    generative models will need a great deal of computational power.
"""

import os
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline

MODEL_DICT = {# Note these should be changed to the location on your local machine
    "text-generation": {
        'GPTNEO': {
            "key": "EleutherAI/gpt-neo-2.7B",
            "pt_path": "/home/chris/Code/zero_shot_playground/ptfiles/gptneo.pt"
        },
        'GPTJ6B': {
            "key": "EleutherAI/gpt-j-6B",
            "pt_path": "/home/chris/Code/zero_shot_playground/ptfiles/gptj.pt"
        },
        "BLOOM": {
            "key": "bigscience/bloom-560m",
            "pt_path": "/home/chris/Code/zero_shot_playground/ptfiles/bloom560.pt"
        }
    },
    'zero-shot-classification': {
        "DeBerta-v3-large": {
            "key": "MoritzLaurer/DeBERTa-v3-large-mnli-fever-anli-ling-wanli",
            "notes": ""
        },
        "DeBerta-v3-base": {
            "key": "MoritzLaurer/DeBERTa-v3-base-mnli-fever-anli",
            "notes": ""
        },
        "DeBerta-v3-xsmall": {
            "key": "MoritzLaurer/DeBERTa-v3-xsmall-mnli-fever-anli-ling-binary",
            "notes": ""
        }
    },
    'token-classification': {
        'english_pos': {
            'key': 'vblagoje/bert-english-uncased-finetuned-pos',
            'notes': ""
        }
    },
    'text-classification': {
        'roberta-emotion': {
            'key': 'j-hartmann/emotion-english-distilroberta-base',
            'notes': ""
        }
    },
    'ner': {
        'bert-ner': {
            'key': 'dslim/bert-base-NER',
            'notes': ""
        }
    }
}


def get_classifier(task, model, device=0):
    """Helper function returns a classifier
        classifier returns dict of labels, scores, and sequence"""

    classifier = pipeline(task,
                          device=device,
                          use_fast=False,
                          model=MODEL_DICT[task][model]["key"])

    return classifier


def get_generator(task, model_name, device, return_full_text=True):
    """Helper function for creating a generator"""
    pt_path = MODEL_DICT[task][model_name]["pt_path"]

    if not os.path.isfile(pt_path):
        if model_name == "GPTJ6B":
            model = AutoModelForCausalLM.from_pretrained(
                MODEL_DICT[task][model_name]["key"],
                revision="float16",
                torch_dtype=torch.float16,
                # low_cpu_mem_usage=True
            )
        else:
            model = AutoModelForCausalLM.from_pretrained(
                MODEL_DICT[task][model_name]["key"]
            )
        torch.save(model, pt_path)

    model = torch.load(pt_path)

    if model_name == "GPTJ6B":
        tokenizer = AutoTokenizer.from_pretrained(
            MODEL_DICT[task][model_name]["key"], torch_dtype=torch.float16)
    else:
        tokenizer = AutoTokenizer.from_pretrained(
            MODEL_DICT[task][model_name]["key"])

    generator = pipeline(task,
                         model=model,
                         tokenizer=tokenizer,
                         device=device,
                         return_full_text=return_full_text
                         )
    return generator, tokenizer


BOT_DEFAULT_PROMPT = "The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n"
BOT_DEFAULT_STARTER = "Hi, I am your virtual personal assistant. It is a pleasure to talk to you."


class ChatLLM():
    """Interface for chatting with LLM that can be found on Huggingface"""

    def __init__(self, model_tag: str = 'GPTNEO', prompt=BOT_DEFAULT_PROMPT, bot_starter=BOT_DEFAULT_STARTER, max_length=100) -> None:
        self.prompt = prompt
        self.backend = "llm"
        self.bot_starter = bot_starter
        self.conversation = [("AI:", bot_starter)]
        self.gen, self.tokenizer = get_generator(
            "text-generation", model_tag, 0, return_full_text=False)

        self.max_conversation_history = 5
        self.max_length = max_length
        self.end_sequence = "Human:"


    def get_bot_response(self, statement, speaker_id="Human", reset_conversation=False):
        """Get bot response to user input while maintaining the conversation"""
        if reset_conversation:
            self.conversation = [("AI:", self.bot_starter)]
            return self.bot_starter

        self.conversation.append((f"{speaker_id}:", statement))
        # self.conversation.append(("AI:", ""))

        input_prompt = self.prompt + "\n\n" + \
            "\n".join(f"{p[0]}\n{p[1]}\n" for p in self.conversation)
        llm_response = self.query_model(input_prompt)
        self.conversation.append(("AI:", llm_response))

        # print(self.prompt + "\n\n" + "\n".join(f"{p[0]}\n{p[1]}\n" for p in self.conversation))

        # if len(self.conversation)> self.max_conversation_history:
        #     self.conversation = self.conversation[1:]

        return llm_response

    def query_model(self, input_prompt):
        """Queries the model for a response"""
        input_len = len(self.tokenizer(input_prompt)['input_ids'])
        # print(f"Prompting with {input_prompt}")
        query_response = self.gen(input_prompt,
                            max_length=int(input_len + self.max_length),
                            pad_token_id=int(
                                self.tokenizer.convert_tokens_to_ids("\n")),
                            temperature=0.8,
                            eos_token_id=int(
                                self.tokenizer.convert_tokens_to_ids(self.end_sequence))
                            )[0]["generated_text"]
        # print(f"LLM Raw Output:\n {query_response}\n Finished LLM Output\n")

        try:
            responses = query_response.split("\n")
            for ind, resp in enumerate(responses):
                if "AI:" in resp:
                    llm_response = responses[ind+1]
                    break
            return llm_response
        except Exception as exc:
            print(exc)
            print(f"Prompting with {input_prompt}")
            print(f"LLM Raw Output:\n {query_response}\n Finished LLM Output\n")
            return None


class ClassifyLLM():
    """Classifier will classify input
    
    Unlike the OpenAI models, this uses an actual classification pipeline.
    """
    def __init__(self, model_tag="DeBerta-v3-large", label_thresh=.2) -> None:
        self.model = get_classifier(
            "zero-shot-classification", model_tag, device=0)
        self.pos_model = get_classifier("token-classification", "english_pos")
        self.label_thresh = label_thresh

    def classify(self, statement, classes, question="Should the prior statement be classified as"):
        """classify statment according to classes provided"""
        results = self.model(
            statement, classes, hypothesis_template=question, multi_label=False)
        num_labels_returned = 1
        # for score in results["scores"]:
        #     if score > self.label_thresh:
        #         num_labels_returned += 1
        ordered_labels = results["labels"][:num_labels_returned]
        return ordered_labels

    def process_pos(self, statement):
        """Uses a pretrained POS Classifier"""
        pos = self.pos_model(statement)
        pos = [(d["word"], d["entity"]) for d in pos]
        print("\nPart of speech tags:", pos, "\n")
        return pos


if __name__ == "__main__":
    print(BOT_DEFAULT_PROMPT)
    print(BOT_DEFAULT_STARTER)
    bot = ChatLLM()
    classifier = ClassifyLLM()
    while True:
        speaker = input("Speaker: ")
        if speaker == "debug":
            print(bot.conversation)
        user_input = input("Says: ")
        response = bot.get_bot_response(user_input, speaker_id=speaker)
        classification = classifier.classify(user_input, ["positive", "negative", "neutral"], question="This is an example of {}")
        print(response, classification)
        keep = input("keep response? (y/n)")
        if keep == "n":
            bot.conversation.pop(-1)
