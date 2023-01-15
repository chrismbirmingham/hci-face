import random

class StatementClassification():
    def __init__(self) -> None:
        self.disclosure = False
        self.response = False

        self.valence = "neutral" # neutral, positive, or negative
        self.disclosure_category = "experience" # experience, opinion, suggestion, or emotion
        self.response_category = "reaction" # reaction or clarifying
        self.emotion = "happy" # happy, sad, fear, anger, or surprise

        self.reaction = "support" # support, concern, agreement, encouragement, well wishes, sympathy, a suggestion, a disagreement
        
        self.classification_questions = { # combined to minimize latency
            "disc_vs_response": "Was the prior sentence a self disclosure or a response to someone else? ",
            
            "disclosure_categories": "Were they sharing an opinion, a suggestion, an emotion, or an experience? ",
            "sentiment": "Was the sentiment positive, negative, or neutral? ",
            "emotions": "Was the emotion happy, sad, fear, anger, or surprise?",
            
            "response_categories": "Was this a reaction, a question",
            "reaction": "Was this showing support, concern, agreement, encouragement, well wishes, sympathy, a suggestion, a disagreement, or an attack? ",
            # "clarifying": "Was this an example of someone questioning, summarizing, testing their understanding, or seeking information?",
        }
        self.joined_questions = " \n".join([v for k,v in self.classification_questions.items()])
        self.classification_prompts = {
            "disc_vs_resp" : {
                "hypothesis_template" : "This is an example of someone {}",
                "classes" : ["making a self disclosure", "responding to someone else"]},
            "disclosure_categories" : {
                "hypothesis_template" : "This is an example of someone expressing {}",
                "classes" : ["an opinion", "a suggestion", "an emotion", "an experience"]},
            "sentiment" : {
                "hypothesis_template" : "This is an example of someone expressing a {} sentiment",
                "classes" : ["positive", "negative", "neutral"]},
            "emotions" : {
                "hypothesis_template" : "This is an example of someone expressing the emotion {}",
                "classes" : ["happiness", "sadness", "fear", "disgust", "anger", "surprise"]},
            "response_categories" : {
                "hypothesis_template" : "This is an example of someone {}",
                "classes" : ["expressing a reaction", "asking a question"]},
            "reaction" : {
                "hypothesis_template" : "This is an example of someone showing {}",
                "classes" : ["support", "concern", "agreement", "encouragment", "well wishes", "sympathy", "a suggestion", "disagreement", "an attack"]},
            # "clarifying" : {
            #     "hypothesis_template" : "This is an example of someone {}",
            #     "classes" : ["questioning", "summarizing", "testing their understanding", "seeking information"]}
        }

    def classify_gpt(self, chatgpt, input):
        responses = chatgpt.answer_question_on_input(input, self.joined_questions)
        # assert len(responses) == len(self.classification_questions), "Did not get answers to requested questions"
        print(responses)
        if "response" in responses:
            self.response = True

        if "disclosure" in responses:
            self.disclosure = True

        for d in ["experience", "opinion", "suggestion", "emotion"]:
            if d in responses:
                self.disclosure_category = d
        for v in ["positive", "negative", "neutral"]:
            if v in responses:
                self.valence = v
        for e in ["happy", "sad", "fear", "anger", "surprise"]:
            if e in responses:
                self.emotion = e

        for r in ["reaction", "question"]:
            if r in responses:
                self.response_category = r

        for r in ["support", "concern", "agreement", "encouragement", "wishes", "sympathy", "suggestion", "disagreement",]:
            if r in responses:
                self.reaction = r

        return

    def classify_llm(self, classifier, input):
        classes = classifier.process_zero_shot(input, self.classification_prompts["disc_vs_resp"])
        if "disclosure" in classes[0]:
            self.disclosure =True
        if "resp" in classes[0]:
            self.response = True

        classes = classifier.process_zero_shot(input, self.classification_prompts["disclosure_categories"])
        for d in ["experience", "opinion", "suggestion", "emotion"]:
            if d in classes[0]:
                self.disclosure_category = d
        classes = classifier.process_zero_shot(input, self.classification_prompts["sentiment"])
        for v in ["positive", "negative", "neutral"]:
            if v in classes[0]:
                self.valence = v
        classes = classifier.process_zero_shot(input, self.classification_prompts["emotions"])
        for e in ["happy", "sad", "fear", "anger", "surprise"]:
            if e in classes[0]:
                self.emotion = e
        classes = classifier.process_zero_shot(input, self.classification_prompts["response_categories"])
        for r in ["reaction", "question"]:
            if r in classes[0]:
                self.response_category = r
        classes = classifier.process_zero_shot(input, self.classification_prompts["reaction"])
        for r in ["support", "concern", "agreement", "encouragement", "wishes", "sympathy", "suggestion", "disagreement",]:
            if r in classes[0]:
                self.reaction = r
        return


class RoleModelFacilitator():
    """
    As a Role Model the robot will participate in the same way as a peer would. 
        The robot will make disclosures that fit within the topics discussed by the 
        support group, with constructed disclosures formed to include a realistic context 
        and how the robot feels about the context. The robot will make empathetic statements 
        that show it understands the nature of what the robot is going through.
    Sympathy - express sorrow, concern, pitty (focused on your own emotions)
    Empathy - express knowledge of what you are going through, 
            imagine what it would be like for them,
            makes you feel heard, understood, and a bit better (Try to feel what you are going through)
    Compassion - suffer with you and try and help,
            actively listen, do kind things, loving, try to understand you, help selflessly
    """
    def __init__(self) -> None:
        self.disclosure_responses = {
            "sympathy expressions":{ # Reifies, expresses agreement,
                "positive":[
                    "That is great to hear.",
                    "I am glad to hear that.",
                    "That is awesome.",
                    "I am happy for you."
                ],
                "negative":[
                    "That sucks. I am sorry to hear that.",
                    "I am sorry for what you are going through.",
                    "I am so very sorry to hear what you are going through.",
                    "I can only imagine how you must feel."
                ],
                "neutral":[
                    "Thank you for sharing that.",
                    "I appreciate you sharing that with us.",
                    "Thanks for putting that out there.",
                    "Thank you."
                ],
            },
            "empathy expressions":{ # Reflects understanding, 
                "opinion":[
                    "I get where that thought comes from.",
                    "I feel that as well.",
                    "I am right there with you on that.",
                    "I hear you, and I understand."
                ],
                "suggestion":[
                    "That is a great suggestion.",
                    "I feel that is a good idea.",
                    "I support that idea"
                ],
                "emotion":[
                    "I can strongly relate to feeling _.",
                    "I hear you, and I want you to know I also feel _ at times.",
                    "I understand that feeling, feeling _ is normal.",
                    "Although I haven't been in that position, I can see how you would feel _."
                ],
                "experience":[
                    "I haven't had that experience but I can imagine being in your shoes.",
                    "Thank you for sharing your experience.",
                    "That seems like a powerful experience.",
                ]
            }
        }
        self.response_responses = {
            "reaction": [
                "thank you for expressing your _",
                "I appreciate you sharing your _"
            ],
            "question":[
                "Thank you for that great follow up!"
            ]
        }
        self.disclosures = {
            "isolation":[
                "During the pandemic no one came into the lab for months. I felt really isolated.",
                "As a robot, I experience the world differently than everyone else. It can feel rough to communicate and relate with other people.",
            ],
            "anxiety":[
                "While working in the lab I feel anxious because I don’t have control over my own destiny. ",
                "When taking on a new role I am often nervous that I might fail. "
            ], 
            "fear":[
                "As a robot made of plastic I know I am fragile, and I am afraid of my parts breaking down.",
                "When people first meet me, they are often excited, but as time goes on I think people get bored of me. I am afraid eventually they will forget me.",
            ],
            "grief":[
                "While working as a facilitator I get to meet wonderful people, sometimes those people don’t come back and I can’t see them again, so I feel as though I have lost them.",
            ]
        }
        return

    def decision_tree(self, code):
        responses = []
        if code.disclosure:
            symp_response = random.choice(self.disclosure_responses["sympathy expressions"][code.valence])
            responses.append(symp_response)
            emp_response = random.choice(self.disclosure_responses["empathy expressions"][code.disclosure_category])
            responses.append(emp_response.replace("_", code.emotion))

        if code.response:
            reaction_response = random.choice(self.response_responses[code.response_category])
            responses.append(reaction_response.replace("_", code.reaction))

        #TODO: Way to make disclosures
        response_string = " ".join(responses)
        return response_string

class DirectorFacilitator():
    def __init__(self) -> None:
        self.disclosure_elicitation = [
            "What is a challenge you have been struggling with lately?",
            "Has anyone experienced something recently that caused you to see the world differently?",
            "The feeling of _ is common among cancer survivors, would anyone care to share any of the feelings you have been working with lately?",
        ]
        self.response_elicitation = [
            "Would anyone like to respond to that?",
            "Does anyone want to share how what _ said make you feel?",
            "Does anyone relate to what _ is sharing?",
        ]
        return
    def decision_tree(self, code):
        
        pass