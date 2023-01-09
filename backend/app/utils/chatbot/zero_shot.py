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


class Bot():
    def __init__(self, bot: str = 'GPTNEO', prompt = None, bot_start=None, max_length = 100) -> None:
        self.max_conversation_history = 5
        self.bot_default = "bot: Hi, I am your virtual personal mental health assistant. How are you doing today?"
        self.prompt = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."
        self.bot_start = bot_start

        self.gen, self.tokenizer = get_generator("text-generation", bot, 0, return_full_text=False)

        if prompt: self.prompt = prompt
        if bot_start: self.bot_default = bot_start
        self.bot_response = self.bot_default

        self.conversation = []
        self.max_length = max_length
        self.end_sequence = "\n"
        return

    def get_bot_response(self, user_input, reset_conversation=False):
        if reset_conversation:
            self.conversation = [(self.bot_default, f"user: {user_input}")]
        else: 
            self.conversation.append((self.bot_response, f"user: {user_input}"))

        bot_input = self.prompt + "\n\n" + "\n".join(f"{p[0]}\n{p[1]}\n" for p in self.conversation)

        input_len = len(self.tokenizer(bot_input)['input_ids'])

        response = self.gen(bot_input, 
                max_length=int(input_len + self.max_length),
                pad_token_id=int(self.tokenizer.convert_tokens_to_ids(self.end_sequence)),
                temperature= 0.8,
                eos_token_id = int(self.tokenizer.convert_tokens_to_ids(self.end_sequence))
            )[0]["generated_text"]

        responses = response.split("\n")
        for r in responses:
            if r != "" and "bot:" in r:
                self.bot_response = r
                break

        print(self.prompt + "\n\n" + "\n".join(f"{p[0]}\n{p[1]}\n" for p in self.conversation))

        if len(self.conversation)> self.max_conversation_history:
            self.conversation = self.conversation[1:]

        return self.bot_response[4:]



disc_vs_resp = {
    "hypothesis_template" : "This is an example of someone {}",
    "classes" : ["making a self disclosure", "responding to someone else"]
}

disclosure_categories = {
    "hypothesis_template" : "This is an example of someone expressing {}",
    "classes" : ["an opinion", "a suggestion", "an emotion", "an experience"]
}

sentiment = {
    "hypothesis_template" : "This is an example of someone expressing a {} sentiment",
    "classes" : ["positive", "negative", "neutral"]
}

emotions = {
    "hypothesis_template" : "This is an example of someone expressing the emotion {}",
    "classes" : ["happiness", "sadness", "fear", "disgust", "anger", "surprise"]
}

response_categories = {
    "hypothesis_template" : "This is an example of someone {}",
    "classes" : ["expressing a reaction", "asking a question"]
}

reaction = {
    "hypothesis_template" : "This is an example of someone showing {}",
    "classes" : ["support", "concern", "agreement", "encouragment", "well wishes", "sympathy", "a suggestion", "disagreement", "an attack"]
}

clarifying = {
    "hypothesis_template" : "This is an example of someone {}",
    "classes" : ["questioning", "summarizing", "testing their understanding", "seeking information"]
}

def process_zero_shot(model, statement, d):
    results = model(statement, d["classes"], hypothesis_template=d["hypothesis_template"], multi_label=False)
    num_labels_returned = 0
    label_thresh = .2
    for s in results["scores"]:
        if s>label_thresh:
            num_labels_returned += 1
    ordered_labels = results["labels"][:num_labels_returned]
    return ordered_labels


def main():
    tag = "DeBerta-v3-large"
    test = "I absolutely hate it in San Diego."
    print(test)

    c = get_classifier("token-classification", "english_pos")
    e = get_classifier("text-classification", "roberta-emotion")
    z = get_classifier("zero-shot-classification", tag, device=0)

    while True:
        pos = c(test)
        pos = [(d["word"], d["entity"]) for d in pos]
        print("\nPart of speech tags:", pos, "\n")
        sentiment_results = process_zero_shot(z, test, sentiment)
        print(sentiment_results)
        basic_emotions_results = process_zero_shot(z, test, emotions)
        print(basic_emotions_results)

        emotion = e(test)
        print(f"\nRoberta-emotion identifies the emotion: {emotion[0]['label']}")

        print("\nDeBerta Zero Shot Produces:")

        dvr_results = process_zero_shot(z, test, disc_vs_resp)
        print(dvr_results)

        if 'making a self disclosure' in dvr_results: #disclosure case

            dc_results = process_zero_shot(z, test, disclosure_categories)
            print("disclosure case: " + str(dc_results))
            if "a suggestion" in dc_results:
                print("Thank you for that suggestion")
            else:
                if "negative" in sentiment_results:
                    print("I am so sorry to hear that")
                if "positive" in sentiment_results:
                    print("That sounds great!")
                print(f"thank you for sharing your {dc_results[0][3:]}!")
                print("INSERT RESPONSE GENERATION HERE")


        if 'responding to someone else' in dvr_results:
            rc_results = process_zero_shot(z, test, response_categories)
            print("response case: " + str(rc_results))

            if "expressing a reaction" in rc_results:
                results = process_zero_shot(z, test, reaction)
                print("reaction case: " + str(results))
                print("Thank you for reacting to what was just said")

            if "asking a question" in rc_results:
                results = process_zero_shot(z, test, clarifying)
                print("question case: " + str(results))
                print("that is a great follow up!")



        test = input("\nPlease input another statment to classify: ")

if __name__ == "__main__":
    main()