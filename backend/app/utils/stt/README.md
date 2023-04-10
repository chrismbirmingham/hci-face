# Speech To Text

Originally, I thought it would be pretty cool to build a robust speech to text module on top of the recently released whisper module, and to pair that with a cloud api for when running on edge devices. I did all of this, and then realized that the [python speech_recognition module](https://github.com/Uberi/speech_recognition) had already done a better and more complete job than I could ever hope to acomplish. 

## Installation

See the instructions [here for installing the speech_recognition module](https://github.com/Uberi/speech_recognition). 

For our purposes, this tl;dr should suffice:
```bash
pip install SpeechRecognition
pip install google-cloud-speech
pip install git+https://github.com/openai/whisper.git soundfile

```

For simplicity I will be starting with just two of the Recognizers provided, the local whisper model and the google speech recognizer.



## Getting Started

To transcribe an audio file it is easiest to use the transcriber module.

To run this file directly, do so from the utils directory:
``` py
    cd backend/app/utils/
    python -m stt.transcriber --filename [path/to/file]
```


## Details

Whisper has a variety of model sizes and model types. Currently we default to the english models but it is relatively simple to set up other languages as well.

For full details on all modules please see the References in the documentation.

The google api only supports files up to 20MB so longer recordings may fail.

Whisper gives punctuation, basic punctuation has been added to google.



## Roadmap
<!-- --8<-- [start:sttroadmap] -->
The current STT module is stable, but future plans include:


- [ ] Live diarization
    - The diarization has been removed for now, future plans will bring it back with better support.
<!-- --8<-- [end:sttroadmap] -->
