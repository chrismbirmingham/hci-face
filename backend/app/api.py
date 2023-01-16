from fastapi import FastAPI, WebSocket, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, PlainTextResponse
from sse_starlette.sse import EventSourceResponse
from dotenv import load_dotenv
import asyncio

from app import FacilitatorChat
from app.utils.stt.whisper_stt import Transcriber
from app.utils.stt.websocket_processor import WebSocketAudioProcessor
from app.utils.tts.coqui_tts import Speak
# from app.utils.chatbot.zero_shot import ChatLLM, ClassifyLLM
# from app.utils.chatbot.chatgpt import ChatGPT
from app.utils.tts.viseme_generator import VisemeGenerator


# First bytes are needed because the opus header gets messed up when passing audio data by a websocket
common_hallucinations = ["        you", "       You", "          Thanks for watching!", "  Thank you for watching!", "        THANK YOU FOR WATCHING!", "   THANKS FOR WATCHING!"]
# PROMPT = "The following is a conversation with an AI assistant that can have meaningful conversations with users. The assistant is helpful, empathic, and friendly. Its objective is to make the user feel better by feeling heard. With each response, the AI assistant prompts the user to continue the conversation naturally."

load_dotenv()

vg = VisemeGenerator("phoneme-viseme_map.csv")
tts = Speak()
stt = Transcriber(model_size="small")
wsap = WebSocketAudioProcessor(queue_length=5, rms_multiplier=1.2)
# bot = ChatLLM(bot="GPTNEO",prompt=PROMPT)
# bot = ChatGPT(prompt=PROMPT)
bot = FacilitatorChat(backend="gpt")
print("Setup Complete")
FACE_CONTROL_QUEUE = {
    "expression":[],
    "eye_aus":[],
    "mouth_aus":[],
    "brow_aus":[],
}

VIZEME_QUEUE = []
VISEME_DELAY = .01  # second
RETRY_TIMEOUT = 15000  # milisecond

app = FastAPI(debug=False)


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
    # WARNING if you have multiple face tabs open, it will split the 
    # visemes sent to each one
    async def event_generator():
        global VIZEME_QUEUE
        while True:
            global VISEME_DELAY
            # If client closes connection, stop sending events
            if await request.is_disconnected():
                # print("Disconnected")
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
                # print("Disconnected")
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
async def websocket_endpoint(websocket: WebSocket):
    """Continuously Open STT, returns on completed phrases, when pauses are detected"""
    await websocket.accept()

    try:
        while True:
            data = await websocket.receive_bytes()
            speech_segment = wsap.process_bytes(data)
            if speech_segment:
                transcribed_text = stt.transcribe_clip(speech_segment)
                hallucination = False
                for h in common_hallucinations:
                    if transcribed_text in h:
                        hallucination=True
                if not hallucination:
                    print("Collected speech length: ", len(speech_segment))
                    print("Transcribed text: ", transcribed_text)
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
    if bot.backend == "gpt":
        bot.bot.conversation[-1] = "AI: " + text
    if bot.backend == "llm":
        bot.bot.conversation[-1] = ("AI:", text)
    global VIZEME_QUEUE
    global VISEME_DELAY
    out, speaking_time = tts.synthesize_wav(text, speaker_id, style_wav)
    viseme_set = vg.get_visemes(text)
    # print(speaking_time, len(viseme_set))
    viseme_length = (speaking_time) / (len(viseme_set)+1)
    VISEME_DELAY = viseme_length
    VIZEME_QUEUE += viseme_set

    return StreamingResponse(out, media_type="audio/wav")


@app.get("/api/bot_response")
def generate_response(text: str, speaker: str, reset_conversation: bool):
    """Generates a bot response"""
    # expresions = ["neutral", "joy", "sad", "surprise"]#,"angry", "disgusted",  "fear"]
    # e = random.choice(expresions)
    # out_text = "This is placeholder text for testing"
    print(f"Face is asking for bot response to {text} from {speaker}")
    tree_response, bot_response = bot.get_bot_response(text, speaker, reset_conversation)
    joined_response = f"{tree_response}&&&{bot_response}"
    bot.bot.conversation.pop(-1)
    FACE_CONTROL_QUEUE["expression"].append(bot.sc.emotion)
    return PlainTextResponse(joined_response)
    # print(f"Tree: {tree_response}\nBot: {bot_response}")
    # keep = input("keep response? ((n)o (t)ree or (b)ot)")
    # if "n" in keep:
    #     bot.bot.conversation.pop(-1)
    #     PlainTextResponse("")
    # elif "t" in keep:
        # if bot.backend == "gpt":
        #     bot.bot.conversation[-1] = "AI: " + tree_response
        # if bot.backend == "llm":
        #     bot.bot.conversation[-1] = ("AI:", tree_response)
    #     return PlainTextResponse(tree_response)
    # else:
    #     return PlainTextResponse(bot_response)