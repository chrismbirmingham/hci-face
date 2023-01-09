from fastapi import FastAPI, WebSocket, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, PlainTextResponse
from sse_starlette.sse import EventSourceResponse
from dotenv import load_dotenv
import asyncio
from app.utils.stt.whisper_stt import Transcriber
from app.utils.stt.websocket_processor import WebSocketAudioProcessor
from app.utils.tts.coqui_tts import Speak
from app.utils.chatbot.zero_shot import Bot
from app.utils.chatbot.chatgpt import ChatGPT
from app.utils.tts.viseme_generator import VisemeGenerator
# from typing import Union
# from pydantic import BaseModel
import random


# First bytes are needed because the opus header gets messed up when passing audio data by a websocket
common_hallucinations = ["you", "You", "You", "Thanks for watching!", " Thanks for watching!"]
PROMPT = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."

load_dotenv()

vg = VisemeGenerator("phoneme-viseme_map.csv")
tts = Speak()
stt = Transcriber(model_size="small")
wsap = WebSocketAudioProcessor(queue_length=5)
# bot = Bot(bot="GPTNEO",prompt=PROMPT)
bot = ChatGPT(prompt=PROMPT)

FACE_CONTROL_QUEUE = {
    "expression":[],
    "eye_aus":[],
    "mouth_aus":[],
    "brow_aus":[],
}

VIZEME_QUEUE = []
VISEME_DELAY = .01  # second
RETRY_TIMEOUT = 15000  # milisecond

app = FastAPI()


origins = [
    "http://localhost:3000",
    "localhost:3000"
]

app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"]
)

@app.get("/", tags=["root"])
async def read_root() -> dict:
    return {"message": "Welcome to your bot fastapi."}

@app.get('/api/visemes')
async def viseme_stream(request: Request):
    async def event_generator():
        global VIZEME_QUEUE
        while True:
            global VISEME_DELAY
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                print("Disconnected")
                break

            global VIZEME_QUEUE
            # Checks for new messages and return them to client if any
            if len(VIZEME_QUEUE) >0:
                msg = VIZEME_QUEUE.pop(0)
                # print(msg, VISEME_DELAY)
                response = {
                        "event": "viseme",
                        "id": "message_id",
                        "retry": RETRY_TIMEOUT,
                        "data": msg,
                }
                yield response

            await asyncio.sleep(VISEME_DELAY)

    return EventSourceResponse(event_generator())

@app.get('/api/faceControl')
async def face_control_stream(request: Request):
    async def event_generator():
        while True:
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                print("Disconnected")
                break

            global FACE_CONTROL_QUEUE
            # Checks for new messages and return them to client if any
            for key, q in FACE_CONTROL_QUEUE.items():
                if len(q) >0:
                    msg = q.pop(0)
                    # print(msg)
                    response = {
                            "event": key,
                            "id": "message_id",
                            "retry": RETRY_TIMEOUT,
                            "data": msg,
                    }
                    yield response

            await asyncio.sleep(.03)

    return EventSourceResponse(event_generator())

@app.websocket("/api/stt")
async def websocket_endpoint(websocket: WebSocket, queue_length: int = 3):
    """Continuously Open STT, returns on completed phrases, when pauses are detected"""
    await websocket.accept()

    try:
        while True:
            data = await websocket.receive_bytes()
            speech_segment = wsap.process_bytes(data)
            if speech_segment:
                print("Collected speech length: ", len(speech_segment))
                transcribed_text = stt.transcribe_clip(speech_segment)
                if transcribed_text not in common_hallucinations:
                    await websocket.send_text(transcribed_text)
                else:
                    print("Seems to have been a hallucination")

    except Exception as e:
        raise Exception(f'Could not process audio: {e}')
    finally:
        await websocket.close()

@app.get("/api/tts")
def text_to_speech(text: str, speaker_id: str = "", style_wav: str = ""):
    """Synthesizes wav bytes from text, with a given speaker ID"""
    global VIZEME_QUEUE
    global VISEME_DELAY
    out, speaking_time = tts.synthesize_wav(text, speaker_id, style_wav)
    viseme_set = vg.get_visemes(text)
    viseme_length = (speaking_time) / (len(viseme_set)+1)
    VISEME_DELAY = viseme_length
    VIZEME_QUEUE += viseme_set

    return StreamingResponse(out, media_type="audio/wav")


@app.get("/api/bot_response")
def generate_response(text: str, reset_conversation: bool):
    """Generates a bot response"""
    expresions = ["neutral", "happy", "sad", "angry", "disgusted", "surprised", "fearful"]
    e = random.choice(expresions)
    FACE_CONTROL_QUEUE["expression"].append(e)
    out_text = bot.get_bot_response(text, reset_conversation)
    # out_text = "This is placeholder text for testing"
    print(f"bot bot response: {out_text}")
    return PlainTextResponse(out_text)