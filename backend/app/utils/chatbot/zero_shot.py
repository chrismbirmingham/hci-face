import os
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline

MODEL_DICT = {
    "text-generation":{
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
    'zero-shot-classification':{
        "DeBerta-v3-large" : { 
            "key" : "MoritzLaurer/DeBERTa-v3-large-mnli-fever-anli-ling-wanli",
            "notes" : ""
            },
        "DeBerta-v3-base" : { 
            "key" : "MoritzLaurer/DeBERTa-v3-base-mnli-fever-anli",
            "notes" : ""
            },
        "DeBerta-v3-xsmall" : { 
            "key" : "MoritzLaurer/DeBERTa-v3-xsmall-mnli-fever-anli-ling-binary",
            "notes" : ""
            }
    },
    'token-classification':{
        'english_pos': {
            'key': 'vblagoje/bert-english-uncased-finetuned-pos',
            'notes':""
        }
    },
    'text-classification':{
        'roberta-emotion': {
            'key': 'j-hartmann/emotion-english-distilroberta-base',
            'notes':""
        }
    },
    'ner':{
        'bert-ner': {
            'key': 'dslim/bert-base-NER',
            'notes':""
        }
    }
}

def get_classifier(task, model, device=0):
    # classifier returns dict of labels, scores, and sequence
    classifier = pipeline(task,
                    device=device,
                    use_fast=False,
                    model=MODEL_DICT[task][model]["key"])

    return classifier

def get_generator(task, model_name, device, return_full_text=True):
    pt = MODEL_DICT[task][model_name]["pt_path"]


    if not os.path.isfile(pt):
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
        torch.save(model, pt)

    model = torch.load(pt)

    if model_name == "GPTJ6B":
        tokenizer = AutoTokenizer.from_pretrained(MODEL_DICT[task][model_name]["key"], torch_dtype=torch.float16)
    else: 
        tokenizer = AutoTokenizer.from_pretrained(MODEL_DICT[task][model_name]["key"])
    
    generator = pipeline(task,
                    model=model,
                    tokenizer=tokenizer,
                    device=device,
                    return_full_text=return_full_text
                    )
    return generator, tokenizer

default_prompt = "The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n"
bot_default = "Hi, I am your virtual personal assistant. It is a pleasure to talk to you."

class ChatLLM():
    def __init__(self, LLM: str = 'GPTNEO', prompt=default_prompt, bot_default=bot_default, max_length=100) -> None:
        self.prompt = prompt
        self.bot_default = bot_default
        self.conversation = [("AI:", bot_default)]
        self.gen, self.tokenizer = get_generator("text-generation", LLM, 0, return_full_text=False)

        self.max_conversation_history = 5
        self.max_length = max_length
        self.end_sequence = "Human:"
        return

    def get_bot_response(self, user_input, speaker="Human", reset_conversation=False):
        if reset_conversation:
            self.conversation = [("AI:", self.bot_default)]
            return self.bot_default


        self.conversation.append((f"{speaker}:", user_input))
        # self.conversation.append(("AI:", ""))

        input_prompt = self.prompt + "\n\n" + "\n".join(f"{p[0]}\n{p[1]}\n" for p in self.conversation)
        llm_response = self._queryLLM(input_prompt)
        self.conversation.append(("AI:",llm_response)) 

        # print(self.prompt + "\n\n" + "\n".join(f"{p[0]}\n{p[1]}\n" for p in self.conversation))

        # if len(self.conversation)> self.max_conversation_history:
        #     self.conversation = self.conversation[1:]

        return llm_response

    def _queryLLM(self, input_prompt):
        input_len = len(self.tokenizer(input_prompt)['input_ids'])
        # print(f"Prompting with {input_prompt}")
        response = self.gen(input_prompt, 
                max_length=int(input_len + self.max_length),
                pad_token_id=int(self.tokenizer.convert_tokens_to_ids("\n")),
                temperature= 0.8,
                eos_token_id = int(self.tokenizer.convert_tokens_to_ids(self.end_sequence))
            )[0]["generated_text"]
        # print(f"LLM Raw Output:\n {response}\n Finished LLM Output\n")

        try:
            responses = response.split("\n")
            for i in range(len(responses)):
                r = responses[i]
                if "AI:" in r:
                    llm_response = responses[i+1]
                    break
            return llm_response
        except Exception as e:
            print(e)
            print(f"Prompting with {input_prompt}")
            print(f"LLM Raw Output:\n {response}\n Finished LLM Output\n")
            return


class ClassifyLLM():
    def __init__(self, model_tag="DeBerta-v3-large", label_thresh = .2) -> None:
        self.model = get_classifier("zero-shot-classification", model_tag, device=0)
        self.pos_model = get_classifier("token-classification", "english_pos")
        self.label_thresh = label_thresh

    def process_zero_shot(self, input, d):
        results = self.model(input, d["classes"], hypothesis_template=d["hypothesis_template"], multi_label=False)
        num_labels_returned = 0
        for s in results["scores"]:
            if s>self.label_thresh:
                num_labels_returned += 1
        ordered_labels = results["labels"][:num_labels_returned]
        return ordered_labels

    def process_pos(self, input):
        pos = self.pos_model(input)
        pos = [(d["word"], d["entity"]) for d in pos]
        print("\nPart of speech tags:", pos, "\n")


if __name__ == "__main__":
    print(default_prompt)
    print(bot_default)
    bot = ChatLLM()
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