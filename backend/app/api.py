#!/usr/bin/env python3
"""Exposes interactive bot functionality through FastAPI

The backend consists of modules that expose the functionality needed
for a interactive multi-modal chatbot. This module provides an api to
that functionality that can be served to any frontend application.

The api provides methods for various forms of synchronous and asynchronous
communication
Synchronous vs Asynchronous:
    - Functions that start with 'get' are synchronous.

    - Functions that start with 'run' are asynchronous, and the result are
    returned through functions that start with 'stream'.

    - Functions that start with 'add_to' are semi-synchronous in that they synchronously
    add to a queue which will be asynchronously processed by the face/bot.

The api also allows communication between two different clients, such as how
the WoZ interface can send a gesture to the api, and the api will essentially
forward that gesture to the robot.

example: Typical usage
    As documented in https://fastapi.tiangolo.com/deployment/manually/
    The api can be run from another module:
    ``` py
    uvicorn.run("app.api:app", host="0.0.0.0", port=8000, reload=True, log_level="error")
    ```
    or it can be run directly:
    ``` bash
    uvicorn api:app --host 0.0.0.0 --port 80
    ```
"""

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


LOGS_DIR = "./logs/pilot1"
FACE_CONTROL_QUEUE = {
    "expression":[],
    "behavior":[],
    "viseme": [],
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
GESTURE_QUEUE = []
HUMAN_INPUT = []
VISEME_DELAYS = []  # second
RETRY_TIMEOUT = 15000  # milisecond


l = Logger(folder=LOGS_DIR)
l.log("Beginning Setup")
load_dotenv()
l.log("Setting Up TTS")
tts = Speaker(backend="polly")
l.log("TTS Set Up Complete, Setting up STT")
stt = Transcriber(model_size="small")

l.log("STT Set Up Complete, Setting up Bots")
bot = FacilitatorChat(chat_backend="gpt", classifier_backend="llm")
rmf = RoleModelFacilitator()
df = DirectorFacilitator()
presets = FacilitatorPresets()


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

@app.get("/", tags=["root"])
async def read_root() -> dict:
    return {"message": "Welcome to your bot fastapi."}

@app.get('/api/viseme_stream')
async def stream_viseme(request: Request):
    """Publishes visemes to a subscriber

        Publishes the viseme after the corresponding viseme delay.
        Only publishes when there are visemes in the queue.
        If no subscribers are listening it will not send any messages
        and the queue will continue to grow.

        warning:
            Only publishes each viseme once. If you have multiple subscribers
            (e.g. multiple tabs or multiple devices with the face open) each
            of the subscribers will only recieve a subset of the visemes.

        Args:
            request (Request): Request for event generator when
                API is called.

        Returns:
            EventSourceResponse: Server Sent Event (sse) source.

        Yields:
            Iterator[EventSourceResponse]: Strings for a viseme
                (the desired shape of the mouth)"""
    l.log("/api/visemes: request recieved.")
    async def event_generator():
        while True:
            global VISEME_DELAYS
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global FACE_CONTROL_QUEUE
            # Checks for new messages and return them to client if any
            if len(FACE_CONTROL_QUEUE["viseme"]) > 0:
                msg = FACE_CONTROL_QUEUE["viseme"].pop(0)
                # l.log(f"Viseme msg: {msg}", printnow=True)
                response = {
                        "event": "viseme",
                        "id": "message_id",
                        "retry": RETRY_TIMEOUT,
                        "data": msg,
                }
                yield response
            if len(VISEME_DELAYS)>0:
                sleep_delay = VISEME_DELAYS.pop(0)
                # l.log(f"Sleep Delay: {sleep_delay}", printnow=True)
                await asyncio.sleep(sleep_delay)
            else:await asyncio.sleep(.05)

    return EventSourceResponse(event_generator())

@app.get('/api/face_stream')
async def stream_face(request: Request) -> EventSourceResponse:
    """Publishes face control messages to a subscriber

        Publishes messages as soon as they are added to the queue.
        Message event argument specifies the type of face control
        message being sent, expression or behavior

        Args:
            request (Request): Request for event generator when
                    API is called.

        Returns:
            EventSourceResponse: Server Sent Event (sse) source.

        Yields:
            Iterator[EventSourceResponse]: Strings for a face behavior or
                expression"""
    # l.log("/api/face_stream: request recieved.")
    async def event_generator():
        while True:
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global FACE_CONTROL_QUEUE
            # Checks for new messages and return them to client if any
            for key, msg_queue in FACE_CONTROL_QUEUE.items():
                if key == 'viseme':
                    continue
                if len(msg_queue) >0:
                    data = msg_queue.pop(0)
                    l.log(f"face control message: {data}")
                    response = {
                            "event": key,
                            "id": "message_id",
                            "retry": RETRY_TIMEOUT,
                            "data": data,
                    }
                    yield response

            await asyncio.sleep(.03)

    return EventSourceResponse(event_generator())

@app.get('/api/text_stream')
async def stream_text(request: Request):
    """Publishes text messages to a subscriber

        Publishes messages as soon as they are added to the queue.

        Args:
            request (Request): Request for event generator when
                    API is called.

        Returns:
            EventSourceResponse: Server Sent Event (sse) source.

        Yields:
            Iterator[EventSourceResponse]: Strings of text"""
    l.log("/api/text: request recieved.")
    async def event_generator():
        while True:
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global TEXT_QUEUE
            # Checks for new messages and return them to client if any
            for key, msg_queue in TEXT_QUEUE.items():
                if len(msg_queue) >0:
                    data = msg_queue.pop(0)
                    l.log(f"Text message: {key}: {data}")
                    response = {
                            "event": key,
                            "id": "message_id",
                            "retry": RETRY_TIMEOUT,
                            "data": data,
                    }
                    yield response

            await asyncio.sleep(.03)

    return EventSourceResponse(event_generator())

@app.get("/api/speech")
def get_speech(text: str, speaker_id: str = "") -> StreamingResponse:
    """Synthesizes wav bytes from text, with a given speaker ID

        This text will be spoken immediately after it is generated, so the
        bot is updated with the knowledge that the facilitator is actually
        saying this text.

        Args:
            text (str): Text to e synthesized
            speaker_id (str, optional): ID of the voice to be used. Defaults to "".

        Returns:
            StreamingResponse: Audio stream of the voice saying the text."""
    l.log(f"/api/speech: {text}, {speaker_id}")
    bot.chatbot.accept_response(text)

    global FACE_CONTROL_QUEUE
    global VISEME_DELAYS
    dt_string = l.get_date_str()
    audio_stream, visemes, delays = tts.synthesize(text,
                                                   speaker_id,
                                                   save_path = f"{LOGS_DIR}/{dt_string}.wav")
    VISEME_DELAYS += delays
    FACE_CONTROL_QUEUE["viseme"] += visemes

    return StreamingResponse(audio_stream, media_type="audio/wav")

@app.get("/api/bot_response")
def run_generate_response(text: str, speaker: str,
                      reset_conversation: bool,
                      director_condition: bool
    ) -> PlainTextResponse:
    """Takes input text and generates possible bot responses

        Possible bot responses include generative responses from the chatbot
        as well as controlled responses from the facilitator. The classifications
        used for generating the facilitator response are added as well.
        Additionally, the emotion that was found in the text is mirrored by
        the robots expression if it is in the subset of possible expressions (joy
        sad, surprise)

        warning:
            All of this text is returned asynchronously through the text_stream.
            The default response is set to the bot response.

        Args:
            text (str): Input said by a human.
            speaker (str): Identify of the speaker
            reset_conversation (bool): Whether or not to restart the conversation.
            director_condition (bool): Flag for controlling what type of
                facilitator is used.

        Returns:
            PlainTextResponse: Defaults to generative bot response."""
    l.log(f"/api/bot_response: '{text}', from {speaker}, "
          f"reset_conversation: {reset_conversation}, director_condition: {director_condition}")

    global TEXT_QUEUE

    classifications = bot.get_classifications(text)
    TEXT_QUEUE["classifications"].append(classifications)

    if bot.classification_processor.emotion in ["joy", "sad", "surprise"]:
        l.log(f"Setting face to: {bot.classification_processor.emotion}")
        FACE_CONTROL_QUEUE["expression"].append(bot.classification_processor.emotion)
    else:
        l.log("Setting face to: neutral")
        FACE_CONTROL_QUEUE["expression"].append("neutral")

    facilitator_response = bot.get_facilitator_response(director_condition)
    TEXT_QUEUE["facilitator_response"].append(facilitator_response)

    bot_response= bot.get_bot_response(text, speaker, reset_conversation)
    TEXT_QUEUE["bot_response"].append(bot_response)
    bot.chatbot.reject_response()

    return PlainTextResponse(bot_response)

@app.get("/api/facilitator_presets")
def get_response(mode: str, query: str) -> PlainTextResponse:
    """Returns text presets based on WoZ input

        Mostly useful for controlling the robot during study interactions.
        Does not use the text_stream as reponse is usually directly output.

        Args:
            mode (str): What mode the facilitator is in. Possibilities include
                role_model - for when the robot is leading a session as a role model,
                direcctor - for when the robot is leading a session as a director,
                and facilitator - for when the robot is not in either condition yet.
            query (str): Key for looking up matching text response.

        Returns:
            PlainTextResponse: Text for the faciliatator to say."""
    l.log(f"/api/facilitator_presets: {mode}, {query}")
    if mode == "facilitator":
        to_say = presets.responses[query]
    if mode == "director":
        if query == "disclosure":
            to_say = random.choice(df.disclosure_elicitation)
        if query == "response":
            to_say = random.choice(df.response_elicitation)
    if mode == "role_model":
        if query == "disclosure":
            emotion = random.choice(list(rmf.disclosures.keys()))
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
def add_to_face(text: str, update_type: str) -> PlainTextResponse:
    """Sends desired expression, behavior, or viseme to the face

        Face presets are sent from the WoZ to this API, and are then
        loaded to the queue for the face to read.

        Args:
            text (str): what command to send
            update_type (str): what type of update to send, either expression
                behavior, or viseme.

        Returns:
            PlainTextResponse: returns the requested text."""
    l.log(f"/api/face_presets: {text}, {update_type}")
    if update_type == "expression":
        FACE_CONTROL_QUEUE["expression"].append(text)
    if update_type == "behavior":
        FACE_CONTROL_QUEUE["behavior"].append(text)
    if update_type == "viseme":
        FACE_CONTROL_QUEUE["viseme"].append(text)

    return PlainTextResponse(text)

@app.get("/api/qt_gesture")
def add_to_gesture(text: str) -> PlainTextResponse:
    """Add desired gesture to queue for the robot

        This function provides a passthrough from the WoZ to the robot bridge.

        Args:
            text (str): Name of the gesture

        Returns:
            PlainTextResponse: Returns the specified gesture."""
    l.log(f"/api/qt_gestures: {text}")
    GESTURE_QUEUE.append(text)
    return PlainTextResponse(text)

@app.get("/api/next_gesture")
def return_gesture() -> PlainTextResponse:
    """Get next gesture from the queue.

        This exposes an endpoint for the robot to regularly ping
        in order to fetch the next gesture it should do.

        Returns:
            PlainTextResponse: desired gesture."""
    global GESTURE_QUEUE
    if len(GESTURE_QUEUE)>0:
        gesture = GESTURE_QUEUE.pop()
        l.log(f"/api/next_gesture: {gesture}")
    else: gesture=""
    # l.log(f"/api/next_gesture: {gesture}")
    return PlainTextResponse(gesture)

@app.post("/api/audio")
async def run_transcribe_audio(uploaded_file: UploadFile) -> dict:
    """Perform speech to text on audio file

        transcribes audio from file and adds transcribed text to
        human_speech in the text queue.

        Args:
            uploaded_file (UploadFile): Recorded audio.

        Returns:
            dict: name of the saved audio file."""
    l.log("/api/audio: temp.wav")
    contents = uploaded_file.file.read()
    data_bytes = io.BytesIO(contents)
    audio_clip = AudioSegment.from_file(data_bytes, codec='opus')
    l.log_sound(audio_clip)
    transcription = stt.transcribe_clip(audio_clip)
    if len(transcription) > 0:
        global TEXT_QUEUE
        TEXT_QUEUE["human_speech"].append(transcription)
        l.log(f"Speech Detected: {transcription}")
    return {"filename": "temp.wav"}
