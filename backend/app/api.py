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
import io
from typing import Union

from fastapi import FastAPI, Request, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, PlainTextResponse

from sse_starlette.sse import EventSourceResponse
from dotenv import load_dotenv
from pydub import AudioSegment
from pydantic import BaseModel

try:
    from app.utils import Speaker, Logger, Transcriber, Responder
except ImportError:
    from utils import Speaker, Logger, Transcriber, Responder


LOGS_DIR = "./logs/development2"
FACE_CONTROL_QUEUE = {
    "expression":[],
    "behavior":[],
    "viseme": [],
    "eye_aus":[],
    "mouth_aus":[],
    "brow_aus":[],
}
TEXT_QUEUE = {
    "transcribed_speech":[],
    "bot_response":[],
    "facilitator_response":[],
    "classifications":[],
}
GESTURE_QUEUE = []
HUMAN_INPUT = []
VISEME_DELAYS = []  # second
RETRY_TIMEOUT = 15000  # milisecond


l = Logger(folder=LOGS_DIR)
l.log("Beginning Setup...")
load_dotenv()
l.log("Setting Up TTS...")
tts = Speaker(backend="polly")
l.log("...TTS Set Up Done")
l.log("Setting up STT...")
stt = Transcriber(service="whisper", model_size="base")
l.log("...STT Set Up Done")
l.log("Setting up Bots...")
chatbot = Responder()
l.log("...Setting up Bots Done")


l.log("Conneting API...")
app = FastAPI(debug=True)
l.log("... All Setup Completed")


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
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"]
)

class Conversation(BaseModel):
    speaker: str
    speech: str
    prompt: Union[str, None] = None 
    history: Union[list, None] = None


@app.get("/", tags=["root"])
@l.log_function
async def read_root() -> dict:
    return {"message": "Welcome to fastapi for HCI-Face."}

@app.get('/api/viseme_stream')
@l.log_function
async def stream_viseme(request: Request) -> EventSourceResponse:
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
@l.log_function
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

@app.get('/api/transcribed_speech')
@l.log_function
async def stream_transcription(request: Request) -> EventSourceResponse:
    """Publishes text messages to a subscriber

        Publishes messages as soon as they are added to the queue.

        Args:
            request (Request): Request for event generator when
                    API is called.

        Returns:
            EventSourceResponse: Server Sent Event (sse) source.

        Yields:
            Iterator[EventSourceResponse]: Strings of text"""
    async def event_generator():
        while True:
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global TEXT_QUEUE
            # Checks for new messages and return them to client if any
            if len(TEXT_QUEUE["transcribed_speech"]) >0:
                data = TEXT_QUEUE["transcribed_speech"].pop(0)
                l.log(f"transcribed_speech: {data}")
                response = {
                        "event": "message",
                        "id": "message_id",
                        "retry": RETRY_TIMEOUT,
                        "data": data,
                }
                yield response

            await asyncio.sleep(.03)

    return EventSourceResponse(event_generator())

@app.get('/api/bot_response')
@l.log_function
async def stream_bots(request: Request) -> EventSourceResponse:
    """Publishes text messages to a subscriber

        Publishes messages as soon as they are added to the queue.

        Args:
            request (Request): Request for event generator when
                    API is called.

        Returns:
            EventSourceResponse: Server Sent Event (sse) source.

        Yields:
            Iterator[EventSourceResponse]: Strings of text"""
    async def event_generator():
        while True:
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
                break

            global TEXT_QUEUE
            # Checks for new messages and return them to client if any
            if len(TEXT_QUEUE["bot_response"]) >0:
                data = TEXT_QUEUE["bot_response"].pop(0)
                l.log(f"bot_response: {data}")
                response = {
                        "event": "message",
                        "id": "message_id",
                        "retry": RETRY_TIMEOUT,
                        "data": data,
                }
                yield response

            await asyncio.sleep(.03)

    return EventSourceResponse(event_generator())

@app.get("/api/bot_speech")
@l.log_function
def get_speech(text: str, speaker_id: str = "") -> StreamingResponse:
    """Synthesizes wav bytes from text, with a given speaker ID

        This text will be spoken immediately after it is generated, so the
        bot is updated with the knowledge that the facilitator is actually
        saying this text.

        Args:
            text (str): Text to be synthesized
            speaker_id (str, optional): ID of the voice to be used. Defaults to "".

        Returns:
            StreamingResponse: Audio stream of the voice saying the text."""
    # l.log(f"Function get_speech generating speech for: {text} with speaker: {speaker_id}")
    global FACE_CONTROL_QUEUE
    global VISEME_DELAYS
    dt_string = l.get_date_str()
    audio_stream, visemes, delays = tts.synthesize(text,
                                                   speaker_id,
                                                   save_path = f"{LOGS_DIR}/{dt_string}.wav")
    VISEME_DELAYS += delays
    FACE_CONTROL_QUEUE["viseme"] += visemes

    return StreamingResponse(audio_stream, media_type="audio/wav")

@app.post("/api/conversation")
@l.log_function
def run_conversation(conversation: Conversation):
    """Takes a conversation as input and gets the bot rosponse

    Args:
        conversation (Conversation): Speaker and speech (history and prompt optional)

    Returns:
        PlainTextResponse: Defaults to generative bot response."""
    print(conversation)
    bot_response = chatbot.bot.get_conversation_response(
            conversation.speaker,
            conversation.speech,
            bot_id="AI",
            prompt=conversation.prompt,
            history=conversation.history
        )
    print(bot_response)
    return PlainTextResponse(bot_response)




@app.get("/api/face_presets")
@l.log_function
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
    if update_type == "expression":
        FACE_CONTROL_QUEUE["expression"].append(text)
    if update_type == "behavior":
        FACE_CONTROL_QUEUE["behavior"].append(text)
    if update_type == "viseme":
        FACE_CONTROL_QUEUE["viseme"].append(text)

    return PlainTextResponse(text)

@app.get("/api/qt_gesture")
@l.log_function
def add_to_gesture(text: str) -> PlainTextResponse:
    """Add desired gesture to queue for the robot

        This function provides a passthrough from the WoZ to the robot bridge.

        Args:
            text (str): Name of the gesture

        Returns:
            PlainTextResponse: Returns the specified gesture."""
    GESTURE_QUEUE.append(text)
    return PlainTextResponse(text)

@app.get("/api/next_gesture")
@l.log_function
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
    return PlainTextResponse(gesture)

@app.post("/api/audio")
@l.log_function
async def run_transcribe_audio(uploaded_file: UploadFile) -> dict:
    """Perform speech to text on audio file

        transcribes audio from file and adds transcribed text to
        human_speech in the text queue.

        Args:
            uploaded_file (UploadFile): Recorded audio.

        Returns:
            dict: name of the saved audio file."""
    contents = uploaded_file.file.read()
    data_bytes = io.BytesIO(contents)
    audio_clip = AudioSegment.from_file(data_bytes, codec='opus')
    l.log_sound(audio_clip)
    audio_clip.export(f"temp.wav", format="wav")
    # transcription = stt.transcribe_clip(audio_clip)
    transcription = stt.transcribe_file("temp.wav")
    if len(transcription) > 0:
        if transcription[0] == " ":
            transcription = transcription[1:] + " "
        if transcription[-1] != " ":
            transcription = transcription + " "
        global TEXT_QUEUE
        TEXT_QUEUE["transcribed_speech"].append(transcription)
        l.log(f"Speech Detected: {transcription}")
    return {"filename": "temp.wav"}
