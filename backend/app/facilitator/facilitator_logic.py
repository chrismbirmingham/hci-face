"""All of the core logic for facilitation is here.

These classes are meant to be used together to process
input text and return an appropriate facilitator response.
"""

import random

class StatementClassification():
    """Gets process statement for all classification categories

    There are two ways of getting classification, you can use and explicit classifier
    with a template and a list of classes, or you can use a generator and prompt it
    to answer the questions in the form of sentences.
    
    attributes:
        disclosure (bool)
        response (bool)
        sentiment (str)
        disclosure_category (str)
        response_category (str)
        emotion (str)
        reaction (str)
        clarifying (str)
        classification_obj

    """
    def __init__(self) -> None:
        start_hypothesis = "This is an example of someone"
        start_question = "Was the first sentence an example of someone"
        self.disclosure = False
        self.response = False
        self.sentiment = "neutral"
        self.disclosure_category = "experience"
        self.response_category = "reaction"
        self.emotion = "happy"
        self.reaction = "support"
        self.clarifying = "summarizing"


        self.classification_obj = {
            "disc_vs_resp" : {
                "hypothesis_template" : start_hypothesis + " {}",
                "question_template" : start_question + " {}",
                "full text classes" : ["making a self disclosure", "making a response to someone else"],
                "single word classes" : ["disclosure", "response"],
                "current class": ""},
            "disclosure_category" : {
                "hypothesis_template" : start_hypothesis + " expressing {}",
                "question_template" : start_question + " expressing {}",
                "full text classes" : ["an opinion", "a suggestion", "an emotion", "an experience"],
                "single word classes" : ["opinion", "suggestion", "emotion", "experience"],
                "current class": ""},
            "sentiment" : {
                "hypothesis_template" : start_hypothesis + " expressing a {} sentiment",
                "question_template" : start_question + " expressing a {} sentiment",
                "full text classes" : ["positive", "negative", "neutral"],
                "single word classes" : ["positive", "negative", "neutral"],
                "current class": ""},
            "emotion" : {
                "hypothesis_template" : start_hypothesis + " expressing the emotion {}",
                "question_template" : start_question + " expressing the emotion {}",
                "full text classes" : ["happiness", "sadness", "fear", "disgust",
                                        "anger", "surprise"],
                "single word classes" : ["happiness", "sadness", "fear", "disgust",
                                        "anger", "surprise"],
                "current class": ""},
            "response_category" : {
                "hypothesis_template" : start_hypothesis + " {}",
                "question_template" : start_question + " {}",
                "full text classes" : ["expressing a reaction", "asking a question"],
                "single word classes" : ["reaction", "question"],
                "current class": ""},
            "reaction" : {
                "hypothesis_template" : start_hypothesis + " showing {}",
                "question_template" : start_question + " showing {}",
                "full text classes" : ["support", "concern", "agreement", "encouragment",
                                        "well wishes","sympathy", "a suggestion",
                                        "disagreement", "an attack"],
                "single word classes" : ["support", "concern", "agreement", "encouragment",
                                        "well wishes","sympathy", "suggestion",
                                        "disagreement", "attack"],
                "current class": ""},
            "clarifying" : {
                "hypothesis_template" : start_hypothesis + " {}",
                "question_template" : start_question + " {}",
                "full text classes" : ["questioning", "summarizing", "testing their understanding",
                                       "seeking information"],
                "single word classes" : ["questioning", "summarizing", "testing", "seeking"],
                "current class": ""},
        }


    def classify_gpt(self, chatbot, statement: str):
        """Generate query for openai and process result

            Args:
                chatbot (obj): class instance with classifier.answer_questions method
                statement (str): input statement to be classified
            """
        question_list = []
        for _,val in self.classification_obj.items():
            ftc = val["full text classes"]
            classes_str = ", ".join(ftc[:-1]) + f", or {ftc[-1]}"
            question_list.append(val["question_template"].replace("{}", classes_str))
        # print(question_list)
        joined_questions = " \n".join([f"{idx+1}. {q}" for idx, q in enumerate(question_list)])

        response_sentences = chatbot.classifier.answer_questions(statement, joined_questions)
        answers = response_sentences.split("\n")[1:]
        answers = [a.lower() for a in answers]
        # print(answers)
        assert len(answers) == len(self.classification_obj), (f"Did not get answers {len(answers)}"
                                        f"to requested questions {len(self.classification_obj)}")

        ind = 0
        for k,val in self.classification_obj.items():
            for possible_class in val["single word classes"]:
                if possible_class in answers[ind]:
                    self.classification_obj[k]["current class"] = possible_class
            ind += 1

    def classify_llm(self, chatbot, statement: str):
        """Generate query for huggingface classifier and process result

            Args:
                chatbot (obj): class instance with classify method
                statement (str): input statement to be classified
            """
        for k,val in self.classification_obj.items():
            classes = chatbot.classifier.classify(statement,
                                                   val["full text classes"],
                                                   question=val["hypothesis_template"])
            for possible_class in val["single word classes"]:
                if possible_class in classes[0]:
                    self.classification_obj[k]["current class"] = possible_class

    def get_classifications(self) -> str:
        """Process classifications into a string.

        Also into the parent class object.

        Returns:
            str: text of the classes that have been identified.
        """
        classifications = ", ".join(val["current class"] for _,val in self.classification_obj.items())
        self.disclosure = self.classification_obj["disc_vs_resp"]["current class"] == "disclosure"
        self.response = self.classification_obj["disc_vs_resp"]["current class"] == "response"

        self.sentiment = self.classification_obj["sentiment"]["current class"]
        self.disclosure_category = self.classification_obj["disclosure_category"]["current class"]
        self.response_category = self.classification_obj["response_category"]["current class"]
        self.emotion = self.classification_obj["emotion"]["current class"]
        self.reaction = self.classification_obj["reaction"]["current class"]
        self.clarifying = self.classification_obj["clarifying"]["current class"]
        return classifications


class RoleModelFacilitator():
    """ Class for generating role model facilitation responses.

        As a Role Model the robot will participate in the same way as a peer would.
        The robot will make disclosures that fit within the topics discussed by the
        support group, with constructed disclosures formed to include a realistic context
        and how the robot feels about the context. The robot will make empathetic statements
        that show it understands the nature of what the robot is going through.

        note: Sympathy vs Empathy vs Compassion
            Sympathy - express sorrow, concern, pitty (focused on your own emotions)

            Empathy - express knowledge of what you are going through,
                    imagine what it would be like for them,
                    makes you feel heard, understood, and a bit better
                    (Try to feel what you are going through)

            Compassion - suffer with you and try and help, actively listen, do kind things, loving,
            try to understand you, help selflessly"""
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
            symp_response = random.choice(
                self.disclosure_responses["sympathy expressions"][code.sentiment])
            emp_response = random.choice(
                self.disclosure_responses["empathy expressions"][code.disclosure_category]).replace(
                                                                                "_", code.emotion)
                
            print(f"(RM) Disclosure-->{code.sentiment}-->{code.disclosure_category}-->{code.emotion}")
            responses = [symp_response, emp_response]
        elif code.response:
            follow_up = random.choice([True,False])
            if follow_up:
                reaction_response = random.choice(self.response_responses[code.response_category]).replace("_", code.reaction)
                responses = [reaction_response]
                print(f"(RM) Response-->{code.response_category}-->{code.reaction}")
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
    """ Class for generating director facilitation responses.
    
    The director style doesn't engage directly with participants
    but instead focuses on encouraging participants to respond to 
    one another.
    
    note: Conversational topics for a healthy support group:
        challenges, successes, failures
        family, friends, coworkers
        motivation, goals, emotions
        health, illness, ability, disability
        sleep, exercise, eating
    note: Relevant Emotions:
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
            "Does anyone have any thoughts to share on [TOPIC].",
            "I'd love to hear your thoughts on [TOPIC]."
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
            "Thank you for your willingness to share openly with us.",
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
            print(f"(Dir) Response-->{code.response_category}-->{code.reaction}")
        else:# code.response:
            transition = random.choice(self.disclosure_transitions)
            responses.append(transition)
            topic_sentence = random.choice(self.topic_sentences).replace("[TOPIC]", random.choice(self.topics))
            responses.append(topic_sentence)
            disc_elicitation = random.choice(self.disclosure_elicitation)
            responses.append(disc_elicitation)
            print(f"(Dir) Disclosure-->{code.sentiment}-->{code.disclosure_category}-->{code.emotion}")

        response_string = " ".join(responses)
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
            "Please share your name, what brings you here today, and where you are from.",
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

