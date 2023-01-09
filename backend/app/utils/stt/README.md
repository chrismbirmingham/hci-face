# Speech To Text

We take audio chunks over the websocket and determine if they contain speech in the websocket_processor.py, if they contain speech the segments are combined and sent to the whisper_stt.py module. 