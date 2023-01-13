import openai
import os
# from openai.embeddings_utils import cosine_similarity, get_embedding

openai.api_key = os.getenv("OPENAI_API_KEY")
openai.Model.list()


conversation_prompt = "The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n"

all_questions = "Was the prior sentence a self disclosure or a response to someone else? \nWas the sentiment positive, negative, or neutral? \nWas the emotion happy, sad, fear, anger, surprise?\nWere they sharing an opinion, a suggestion, an emotion, or an experience? \nWas this a reaction, a question, or neither? \nWas this showing support, concern, agreement, encouragement, well wishes, sympathy, a suggestion, a disagreement, or an attack? \nWas this an example of someone questioning, summarizing, testing their understanding, or seeking information?"

cq = {
    "disc_vs_response": "Was the prior sentence a self disclosure or a response to someone else? ",
    "sentiment": "Was the sentiment positive, negative, or neutral? ",
    "emotions": "Was the emotion happy, sad, fear, anger, surprise?",
    "disclosure_categories": "Were they sharing an opinion, a suggestion, an emotion, or an experience? ",
    "response_categories": "Was this a reaction, a question, or neither? ",
    "reaction": "Was this showing support, concern, agreement, encouragement, well wishes, sympathy, a suggestion, a disagreement, or an attack? ",
    "clarifying": "Was this an example of someone questioning, summarizing, testing their understanding, or seeking information?",
}

class ChatGPT():
    def __init__(self, prompt=conversation_prompt) -> None:
        self.prompt = prompt
        self.conversation = [prompt]
        pass

    def converse(self):
        while True:
            i = input("Human: ")
            response = self.get_bot_response(i)
            print(response)

    def get_bot_response(self, user_input, reset_conversation=False):
        if reset_conversation: self.conversation = [self.prompt]
        self.conversation.append("Human: " + user_input)
        self.conversation.append("AI: ")
        prompt = "\n".join(self.conversation)
        # print(prompt)
        response = self.queryAPI(prompt)["choices"][0]["text"]
        response = response.replace("\n",'')
        self.conversation.append(response)
        return response

    def queryAPI(self, prompt, stop=[" Human:", " AI:"], temp=.9, mt=150):
        # Generate a response
        response = openai.Completion.create(
            model="text-davinci-003",
            temperature=temp,
            prompt = prompt,
            max_tokens=mt,
            top_p=1,
            frequency_penalty=0.0,
            presence_penalty=0.6,
            stop=stop
        )
        return response

    def classify(self, input, question):
        prompt = f"{input}\n\n{question}\n\n\n"
        stop = ["\n"]
        return self.queryAPI(prompt, stop=stop, temp=0, mt=80)

        
ps  = []
pr = ["support", "concern", "agreement", "encouragement", "well wishes", "sympathy", "a suggestion", "disagreement", "attack"]
pc = ["questioning", "summarizing", "testing their understanding", "seeking information"]


if __name__ == "__main__":
    # print(conversation_prompt)
    # print("What would you like to start your conversation with?")
    cgpt = ChatGPT()
    # cgpt.converse()
    while True:
        statement = input("---")
        print(statement)
        r = cgpt.classify(statement, all_questions)["choices"][0]["text"]
        l = r.split(".")
        for i in range(len(l)):
            print(i, l[i])

        sentiment = l[1].split(" ")[-1]
        emotion = l[2].split(" ")[-1]
        if "disclosure" in l[0]:
            if "opinion" in l[3]:
                dc = "opinion"
            if "suggest" in l[3]:
                dc = "suggest"
            if "emotion" in l[3]:
                dc = "emotion"
            if "experience" in l[3]:
                dc = "experience"
            print(f"Disclosure of {dc} which was {sentiment}")
            if sentiment == "neutral":
                print(f"Thank you for sharing you {dc}")
            if sentiment == "negative":
                print(f"I am so sorry to hear that!. Thank you for sharing you {dc}. I get how you might feel {emotion}")
            if sentiment == "positive":
                print(f"I am so glad to hear that!. Thank you for sharing you {dc}. I it seems like you feel {emotion}")

        if "response" in l[0]:
            if "reaction" in l[4]:
                rc = "reaction"
                for r in pr:
                    if r in l[5]:
                        reaction = r
                print(f"Thank you for your expressing you {reaction}")
            if "question" in l[4]:
                rc = "question"
                for c in pc:
                    if c in l[6]:
                        clarifying = c
                print(f"Thank you for following up!")
            if "neither" in l[4]:
                rc = "neither"

        e = cgpt.classify(statement, "A good empathetic response would be:")["choices"][0]["text"]
        print(e)

    # for k,v in cq.items():
    #     r = cgpt.classify(input, v)["choices"][0]["text"]
    #     print(k, r)
