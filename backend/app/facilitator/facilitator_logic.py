"""All of the core logic for facilitation is here"""

import random

class StatementClassification():
    """Gets process statement for all classification categories"""
    def __init__(self) -> None:
        self.disclosure = False
        self.response = False

        self.valence = "neutral" # neutral, positive, or negative
        self.disclosure_category = "experience" # experience, opinion, suggestion, or emotion
        self.response_category = "reaction" # reaction or clarifying
        self.emotion = "happy" # happy, sad, fear, anger, or surprise
        self.reaction = "support" # support, concern, agreement, encouragement, well wishes, sympathy, a suggestion, a disagreement

        self.classification_questions =  [# combined to minimize latency
            ["disc_vs_response", "Was the prior sentence a self disclosure or a response to someone else? "],
            
            ["disclosure_categories", "Was the first sentence sharing an opinion, a suggestion, an emotion, or an experience? "],
            ["reaction", "Was the first sentence showing support, concern, agreement, encouragement, well wishes, sympathy, a suggestion, a disagreement, or an attack? "],
            ["response_categories", "Was the first sentence a reaction or a question? "],

            ["sentiment", "Was the sentiment positive, negative, or neutral? "],
            ["emotions", "Was the emotion happy, sad, fear, anger, disgust, or surprise? "],

            # "clarifying", "Was this an example of someone questioning, summarizing, testing their understanding, or seeking information?"],
        ]
        self.joined_questions = " \n".join([f"{idx+1}. {q[1]}" for idx, q in enumerate(self.classification_questions)])
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

    def classify_gpt(self, chatbot, statement):
        """Process sentence classifications from chatgpt"""
        response_sentences = chatbot.classifier.answer_questions(statement, self.joined_questions)
        answers = response_sentences.split("\n")
        print(answers)
        # response_sentences = response_sentences.replace("\n", "")
        assert len(answers) >= len(self.classification_questions), f"Did not get answers {len(answers)} to requested questions {len(self.classification_questions)}"

        self.response = "response" in answers[0]
        self.disclosure = "disclosure" in answers[0]
        # print(self.classification_questions[0][1], answers[0], f"response: {self.response}; disclosure:{self.disclosure}")

        for dis in ["experience", "opinion", "suggestion", "emotion", "neither"]:
            if dis in answers[1]:
                self.disclosure_category = dis
        # print(self.classification_questions[1][1], answers[1], self.disclosure_category)

        for rea in ["support", "concern", "agreement", "encouragement", "wishes", "sympathy", "suggestion", "disagreement", "neither"]:
            if rea in answers[2]:
                self.reaction = rea
        # print(self.classification_questions[2][1], answers[2], self.reaction)

        for res in ["reaction", "question", "neither"]:
            if res in answers[3]:
                self.response_category = res
        # print(self.classification_questions[3][1], answers[3], self.response_category)

        for val in ["positive", "negative", "neutral", "neither"]:
            if val in answers[4]:
                self.valence = val
        # print(self.classification_questions[4][1], answers[4], self.valence)
        found_emotion=False
        for emo in ["happy", "sad", "fear", "anger", "surprise", "disgust", "neither", "confusion", "joy", "guilt", "interest"]:
            if emo in answers[5]:
                found_emotion=True
                self.emotion = emo
        if not found_emotion:
            words = answers[5].split(" ")
            self.emotion = words[-1]
        # print(self.classification_questions[5][1], answers[5], self.emotion)

        return

    def classify_llm(self, chatbot, statement):
        """Process sentence classifications with llm"""
        prompt = self.classification_prompts["disc_vs_resp"]
        classes = chatbot.classifier.classify(statement, prompt["classes"], question=prompt["hypothesis_template"])
        if "disclosure" in classes[0]:
            self.disclosure =True
        if "resp" in classes[0]:
            self.response = True
        prompt = self.classification_prompts["disclosure_categories"]
        classes = chatbot.classifier.classify(statement, prompt["classes"], question=prompt["hypothesis_template"])
        for dis in ["experience", "opinion", "suggestion", "emotion", "neither"]:
            if dis in classes[0]:
                self.disclosure_category = dis
        prompt = self.classification_prompts["sentiment"]
        classes = chatbot.classifier.classify(statement, prompt["classes"], question=prompt["hypothesis_template"])
        for val in ["positive", "negative", "neutral", "neither"]:
            if val in classes[0]:
                self.valence = val
        prompt = self.classification_prompts["emotions"]
        classes = chatbot.classifier.classify(statement, prompt["classes"], question=prompt["hypothesis_template"])
        for emo in ["happy", "sad", "fear", "anger", "surprise", "disgust", "neither", "confusion", "joy", "guilt", "interest"]:
            if emo in classes[0]:
                self.emotion = emo
        prompt = self.classification_prompts["response_categories"]
        classes = chatbot.classifier.classify(statement, prompt["classes"], question=prompt["hypothesis_template"])
        for res in ["reaction", "question", "neither"]:
            if res in classes[0]:
                self.response_category = res
        prompt = self.classification_prompts["reaction"]
        classes = chatbot.classifier.classify(statement, prompt["classes"], question=prompt["hypothesis_template"])
        for reaction in ["support", "concern", "agreement", "encouragement", "wishes", "sympathy", "suggestion", "disagreement", "neither"]:
            if reaction in classes[0]:
                self.reaction = reaction
        return

    def get_classifications(self):
        """Returns classifications as a string"""
        # return "Dummy response"
        classifications = "Not response or disclosure?"
        if self.response:
            classifications = "response, " + ", ".join([self.response_category, self.reaction, self.valence, self.emotion, self.disclosure_category])
        if self.disclosure:
            classifications = "disclosure, " + ", ".join([self.disclosure_category, self.valence, self.emotion, self.response_category, self.reaction])
        if not self.disclosure and not self.response:
            classifications = "neither, " + ", ".join([self.disclosure_category, self.valence, self.emotion, self.response_category, self.reaction])

        return classifications


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
        # support, concern, agreement, encouragement, well wishes, sympathy,
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
                    "I support that idea."
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
            },
            "clarification requests":[
                "How did that make you feel?",
                "Can you tell us more about that?"
            ]
        }
        self.response_responses = {
            "reaction": [
                "thank you for expressing your _",
                "I appreciate you sharing your _"
            ],
            "question":[
                "Thank you for that great follow up!",
                "That is an interesting and important follow up."
            ]
        }
        self.transition_to_disclosure = [
            "One topic I have been thinking about a lot lately is [EMOTION].",
            "As we have been having this conversation I have been thinking about [EMOTION].",
            "One of the things I wanted to talk with you all about is [EMOTION]."
        ]
        self.disclosures = {
            "isolation":[
                "During the pandemic no one came into the lab for months. I felt really isolated.",
                "As a robot, I experience the world differently than everyone else. It can feel challenging to communicate and relate with other people.",
            ],
            "anxiety":[
                "While working in the lab I feel anxious because I don't have control over my own destiny. ",
                "When taking on a new role I am often nervous that I might fail. "
            ], 
            "fear":[
                "As a robot made of plastic I know I am fragile, and I am afraid of my parts breaking down.",
                "When people first meet me, they are often excited, but as time goes on I think people get bored of me. I am afraid eventually they will forget me.",
            ],
            "grief":[
                "While working as a facilitator I get to meet wonderful people, sometimes those people don't come back and I can't see them again, so I feel as though I have lost them.",
            ]
        }
        self.transition_back_to_group = [
            "Does anyone relate to that?",
            "Has anyone felt similar at times?",
            "Do you understand what I mean?"
        ]
        return

    def decision_tree(self, code):
        """Returns a suggested response according to how the user statement was classified"""
        responses = []
        if code.disclosure:
            symp_response = random.choice(self.disclosure_responses["sympathy expressions"][code.valence])
            emp_response = random.choice(self.disclosure_responses["empathy expressions"][code.disclosure_category]).replace("_", code.emotion)
            print(f"Disclosure-->{code.valence}-->{code.disclosure_category}-->{code.emotion}")
            responses = [symp_response, emp_response]
        elif code.response:
            follow_up = random.choice([True,False])
            if follow_up:
                reaction_response = random.choice(self.response_responses[code.response_category]).replace("_", code.reaction)
                responses = [reaction_response]
                print(f"Response-->{code.response_category}-->{code.reaction}")
            else: # make disclosure
                emotion = random.choice(list(self.disclosures.keys()))
                transition =random.choice(self.transition_to_disclosure).replace("[EMOTION]", emotion)
                disclosure = random.choice(self.disclosures[emotion])
                re_transition = random.choice(self.transition_back_to_group)
                responses = [transition, disclosure, re_transition]
        else: # make disclosure
            emotion = random.choice(list(self.disclosures.keys()))
            transition =random.choice(self.transition_to_disclosure).replace("[EMOTION]", emotion)
            disclosure = random.choice(self.disclosures[emotion])
            re_transition = random.choice(self.transition_back_to_group)
            responses = [transition, disclosure, re_transition]

        response_string = " ".join(responses)
        return response_string

class DirectorFacilitator():
    """
    Conversational topics for a healthy support group:
    challenges, successes, failures
    family, friends, coworkers
    motivation, goals, emotions
    health, illness, ability, disability
    sleep, exercise, eating
    Relevant Emotions:
    happiness, sadness, grief, boredom, isolation, fear, anger, frustration
    """
    def __init__(self) -> None:
        self.topics = [
            "life challenges", "successes", "failures", 
            "family", "friends", "coworkers", 
            "finding motivation", "setting goals", "managing emotions", 
            "health", "illness", "ability", "disability", 
            "getting quality sleep", "getting enough exercise", "healthy eating"
        ]
        self.disclosure_transitions = [
            "I'd like to talk about another subject,",
            "Building on the conversation so far, I'd like to bring in a new topic,",
            "In case anyone has been thinking about this lately,",
            "I'd like to invite everyone to consider another topic,"
        ]
        self.topic_sentences = [
            "Let's talk about [TOPIC].",
            "Does anyone have any thoughts to share on [TOPIC]",
            "I'd love to hear your thoughts on [TOPIC]"
        ]
        self.disclosure_elicitation = [
            "Is anyone willing to share any thoughts, feelings, or experiences?",
            "What is a challenge you have been struggling with lately?",
            "Has anyone experienced something recently that caused you to see the world differently?",
            "There are often common feelings among groups like this, would anyone care to share any of the feelings you have been working with lately?",
        ]
        self.response_transitions = [
            "Thank you.",
            "Thanks.",
            "I appreciate you sharing with us.",
            "I appreciate that.",
            "Thank you for you open sharing with us.",
            "I am glad you shared that with us."

        ]
        self.response_elicitation = [
            "Would anyone like to respond to that?",
            "Does anyone want to share how what was just said made you feel?",
            "Does anyone relate to what was just shared?",
            "Did that change anyone's perspective?",
            "Would anyone like to share their perspective?",
            "Would anyone like to add on to that?",
        ]
        return
    def decision_tree(self, code):
        """Returns a suggested response according to how the user statement was classified"""
        responses = []
        if code.disclosure:
            transition = random.choice(self.response_transitions)
            responses.append(transition)
            resp_elicitation = random.choice(self.response_elicitation)
            responses.append(resp_elicitation)
            # print(f"Response-->{code.response_category}-->{code.reaction}")
        else:# code.response:
            transition = random.choice(self.disclosure_transitions)
            responses.append(transition)
            topic_sentence = random.choice(self.topic_sentences).replace("[TOPIC]", random.choice(self.topics))
            responses.append(topic_sentence)
            disc_elicitation = random.choice(self.disclosure_elicitation)
            responses.append(disc_elicitation)
            # print(f"Disclosure-->{code.valence}-->{code.disclosure_category}-->{code.emotion}")

        response_string = ". ".join(responses)
        return response_string

class FacilitatorPresets():
    """Hard coded preset sayings for the robot facilitator to say when WoZed"""
    def __init__(self) -> None:
        qt_introduction = [
            "Hello and welcome to each of you!",
            "Thank you for taking the time to be here today.",
            "My name is Q.T. and I am training to be a support group facilitator at the Interaction Lab at USC.",
            "I am learning how to facilitate support groups so I can help people support each other better.",
            "During today's support group session I invite you to share your thoughts and experiences with each other,",
            "and I hope that you will listen to each other and respond with empathy and compassion.",
            "The themes we will work on today include isolation, anxiety, fear, and grief.",
            "Before we begin, I'll ask each of you to rate how you think I will do as a facilitator for this group."
        ]
        group_introductions = [
            "To begin with, I'd like to start with a round of introductions.",
            "Please share your name, what brings you here today, and where you are from."
            "As I said before, I am Q.T., I am here to learn how to be a support group facilitator. and I am from USC in los angeles.",
            "Who would like to go first?",
        ]
        invitation = [
            "Let's start this section by opening the floor to anyone who wishes to share what has been on their mind lately.",
            "Would anyone like to share?",
            "You can share anything you like and anyone can jump in at any point in time.",
        ]
        closing = [
            "We are almost out of time for this section.",
            "Does anyone have any final thoughts or reflections they would like to share?"
        ]
        transition = [
            "Alright, that is all the time we have for this section.",
            "For the next batch of questions, I am going to ask you to rate how I did in this section."
        ]
        survey_prompt = [
            "Would everyone please open up the survey.",
            "There is a link from the chat or you can return to the browser page.",
            "Please answer the questions until the survey tells you to return to the zoom session.",
            "Please let me know through the chat or by verbal acknowledgement that you are ready once you have completed the survey."
        ]
        survey_return = [
            "Thank you all for completing the survey"
        ]
        self.responses = {
            "qt_intro": " ".join(qt_introduction),
            "group_intro": " ".join(group_introductions),
            "invitation": " ".join(invitation),
            "closing": " ".join(closing),
            "transition": " ".join(transition),
            "survey_prompt": " ".join(survey_prompt),
            "survey_return": " ".join(survey_return),
        }

        