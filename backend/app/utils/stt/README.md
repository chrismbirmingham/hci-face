# Speech To Text

We use a downloaded copy of the whisper speech-to-text model and pyannote to diarize the output. 

## Installation

To run this sub package directly you will need to have pyaudio, whisper and pyannote installed. Please follow the directions from [MIT for pyaudio](https://people.csail.mit.edu/hubert/pyaudio/), the directions from [openai for whisper](https://github.com/openai/whisper), and [these directions for pyanote](https://github.com/pyannote/pyannote-audio)

## Getting Started

To transcribe an audio file it is easiest to use the transcriber module.

To run this file directly, do so from the utils directory:
``` py
    cd backend/app/utils/
    python -m stt.transcriber --filename [path/to/file]
```

### Backend Components

The backend components (whisper, diarization, and recording) are available as self contained modules for individual usage as well.

## Details

Whisper has a variety of model sizes and model types. Currently we default to the english models but it is relatively simple to set up other languages as well.

For full details on all modules please see the References in the documentation.

## Roadmap
<!-- --8<-- [start:sttroadmap] -->
The current STT module is stable, but future plans include:

- [ ] Live diarization
    - Diarization is currently only available for transcribing full audio files. Future work will enable integrate the stt more closely with the conversation, so that speakers can be tracked while they are transcribed from audio clips.
- [ ] Stream handling
    - Stream segmentation has been moved to the frontend for now. Future work will enable command line support for transcribing live streams.
- [ ] Cloud STT & Diarization
    - Currently the stt module is entirely local, as it doesn't require heavy computational resources or a GPU. In the future we will add support for cloud transcription to better support use on lightweight/edge devices.
<!-- --8<-- [end:sttroadmap] -->
