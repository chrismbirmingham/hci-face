"""Exposes interactive bot functionality through FastAPI"""

import asyncio
import random
import io

from fastapi import FastAPI, Request, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, PlainTextResponse

from sse_starlette.sse import EventSourceResponse
from dotenv import load_dotenv
from pydub import AudioSegment

from app.utils import Speaker, Logger, Transcriber
from app.facilitator import FacilitatorChat, RoleModelFacilitator
from app.facilitator import DirectorFacilitator, FacilitatorPresets

common_hallucinations = ["        you",
     "       You",
     "          Thanks for watching!",
     "  Thank you for watching!",
     "        THANK YOU FOR WATCHING!",
     "Thanks for watching! Don't forget to like, comment and subscribe!"
     "   THANKS FOR WATCHING!"]
# PROMPT = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."
LOGS_DIR = "./logs/testing"

l = Logger(folder=LOGS_DIR)
l.log("Beginning Setup")
load_dotenv()
l.log("Setting Up TTS")
tts = Speaker(backend="coqui")
l.log("TTS Set Up Complete, Setting up STT")
stt = Transcriber(model_size="small")

l.log("STT Set Up Complete, Setting up Bots")
bot = FacilitatorChat(backend="gpt")
rmf = RoleModelFacilitator()
df = DirectorFacilitator()
presets = FacilitatorPresets()

FACE_CONTROL_QUEUE = {
    "expression":[],
    "behavior":[],
    "eye_aus":[],
    "mouth_aus":[],
    "brow_aus":[],
}
TEXT_QUEUE = {
    "human_speech":[],
    "bot_response":[],
    "facilitator_response":[],
    "classifications":[],
}
VIZEME_QUEUE = []
GESTURE_QUEUE = []
HUMAN_INPUT = []
VISEME_DELAYS = []  # second
RETRY_TIMEOUT = 15000  # milisecond

l.log("Conneting API")
app = FastAPI(debug=True)
l.log("Setup Complete")


origins = [
    "http://localhost:3000",
    "localhost:3000",
    "http://localhost:3001",
    "localhost:3001",
    "http://localhost:3002",
    "localhost:3002",
    "*"
]

app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"]
)
print("API Setup Complete")


@app.get("/", tags=["root"])
async def read_root() -> dict:
    return {"message": "Welcome to your bot fastapi."}

@app.get('/api/viseme_stream')
async def viseme_stream(request: Request):
    # l.log(f"/api/visemes: request recieved.")
    # WARNING if you have multiple face tabs open, it will split the 
    # visemes sent to each one
    async def event_generator():
        global VIZEME_QUEUE
        while True:
            global VISEME_DELAYS
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global VIZEME_QUEUE
            # Checks for new messages and return them to client if any
            if len(VIZEME_QUEUE) > 0:
                msg = VIZEME_QUEUE.pop(0)
                # l.log(f"Viseme msg: {msg}", printnow=False)
                response = {
                        "event": "viseme",
                        "id": "message_id",
                        "retry": RETRY_TIMEOUT,
                        "data": msg,
                }
                yield response
            if len(VISEME_DELAYS)>0:
                w = VISEME_DELAYS.pop(0)
                await asyncio.sleep(w)
            else:await asyncio.sleep(.05)

    return EventSourceResponse(event_generator())

@app.get('/api/face_stream')
async def face_stream(request: Request):
    # l.log(f"/api/face_stream: request recieved.")
    async def event_generator():
        while True:
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global FACE_CONTROL_QUEUE
            # Checks for new messages and return them to client if any
            for key, q in FACE_CONTROL_QUEUE.items():
                if len(q) >0:
                    msg = q.pop(0)
                    l.log(f"face control message: {msg}")
                    response = {
                            "event": key,
                            "id": "message_id",
                            "retry": RETRY_TIMEOUT,
                            "data": msg,
                    }
                    yield response

            await asyncio.sleep(.03)

    return EventSourceResponse(event_generator())

@app.get('/api/text_stream')
async def text_stream(request: Request):
    # l.log(f"/api/human_text_stream: request recieved.")
    async def event_generator():
        while True:
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global TEXT_QUEUE
            # Checks for new messages and return them to client if any
            for key, q in TEXT_QUEUE.items():
                if len(q) >0:
                    msg = q.pop(0)
                    l.log(f"face control message: {key}: {msg}")
                    response = {
                            "event": key,
                            "id": "message_id",
                            "retry": RETRY_TIMEOUT,
                            "data": msg,
                    }
                    yield response

            await asyncio.sleep(.03)

    return EventSourceResponse(event_generator())

@app.get("/api/speech")
def text_to_speech(text: str, speaker_id: str = ""):
    """Synthesizes wav bytes from text, with a given speaker ID"""
    l.log(f"/api/speech: {text}, {speaker_id}")
    if bot.backend == "gpt":
        bot.bot.conversation[-1] = "AI: " + text
    if bot.backend == "llm":
        bot.bot.conversation[-1] = ("AI:", text)

    global VIZEME_QUEUE
    global VISEME_DELAYS

    audio_stream, visemes, delays = tts.synthesize(text, speaker_id)
    VISEME_DELAYS += delays
    VIZEME_QUEUE += visemes

    return StreamingResponse(audio_stream, media_type="audio/wav")

@app.get("/api/bot_response")
def generate_response(text: str, speaker: str, reset_conversation: bool, director_condition: bool):
    """Generates a bot response"""
    l.log(f"/api/bot_response: '{text}', from {speaker}, reset_conversation: {reset_conversation}, director_condition: {director_condition}")
    global TEXT_QUEUE

    classifications = bot.get_classifications(text)
    TEXT_QUEUE["classifications"].append(classifications)
    if bot.sc.emotion in ["joy", "sad", "surprise"]:
        l.log(f"Setting face to: {bot.sc.emotion}")
        FACE_CONTROL_QUEUE["expression"].append(bot.sc.emotion)
    else:
        l.log("Setting face to: neutral")
        FACE_CONTROL_QUEUE["expression"].append("neutral")

    facilitator_response = bot.get_facilitator_response(director_condition)
    TEXT_QUEUE["facilitator_response"].append(facilitator_response)

    bot_response= bot.get_bot_response(text, speaker, reset_conversation)
    TEXT_QUEUE["bot_response"].append(bot_response)
    bot.bot.conversation.pop(-1)
    
    return PlainTextResponse(bot_response)

@app.get("/api/facilitator_presets")
def return_response(mode: str, query: str):
    """Returns an existing bot response"""
    l.log(f"/api/facilitator_presets: {mode}, {query}")
    if mode == "facilitator": to_say = presets.responses[query]
    if mode == "director":
        if query == "disclosure":
            to_say = random.choice(df.disclosure_elicitation)
        if query == "response":
            to_say = random.choice(df.response_elicitation)
    if mode == "role_model":
        if query == "disclosure":
            emotion = random.choice(rmf.disclosures.keys())
            transition =random.choice(rmf.transition_to_disclosure).replace("[EMOTION]", emotion)
            disclosure = random.choice(rmf.disclosures[emotion])
            re_transition = random.choice(rmf.transition_back_to_group)
            responses = [transition, disclosure, re_transition]
            to_say = " ".join(responses)
        if query == "response":
            print("getting response")
            to_say = random.choice(rmf.disclosure_responses["sympathy expressions"]["neutral"])
            print(to_say)
            to_say2 = random.choice(rmf.disclosure_responses["clarification requests"])
            print(to_say,to_say2)
            to_say = to_say + ". " + to_say2
    l.log(f"facilitator_presets response: {to_say}")
    return PlainTextResponse(to_say)
    
@app.get("/api/face_presets")
def update_face(text: str, update_type: str):
    """Returns an existing bot response"""
    l.log(f"/api/face_presets: {text}, {update_type}")
    if update_type == "expression":
        FACE_CONTROL_QUEUE["expression"].append(text)
    if update_type == "behavior":
        FACE_CONTROL_QUEUE["behavior"].append(text)
    if update_type == "viseme":
        VIZEME_QUEUE.append(text)

    return PlainTextResponse(text)

@app.get("/api/qt_gesture")
def add_gesture(text: str):
    l.log(f"/api/qt_gestures: {text}")
    GESTURE_QUEUE.append(text)
    return PlainTextResponse(text)

@app.get("/api/next_gesture")
def return_gesture():
    global GESTURE_QUEUE
    if len(GESTURE_QUEUE)>0:
        g = GESTURE_QUEUE.pop()
        l.log(f"/api/next_gesture: {g}")
    else: g=""
    return PlainTextResponse(g)
    
@app.post("/api/audio")
async def create_upload_file(uploadedFile: UploadFile):
    contents = uploadedFile.file.read()
    data_bytes = io.BytesIO(contents)
    audio_clip = AudioSegment.from_file(data_bytes, codec='opus')
    l.log_sound(audio_clip)
    transcription = stt.transcribe_clip(audio_clip)
    global TEXT_QUEUE
    TEXT_QUEUE["human_speech"].append(transcription)
    l.log(f"Speech Detected: {transcription}")
    return {"filename": "temp.wav"}


