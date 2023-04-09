# Text To Speech

The text to speech is done through an open source library courtesy of coqui.ai. This produces a wav file for a given text input. Many models are available through coqui, but we utilize the VITS end-to-end model trained on the VCTK multi-speaker dataset, as this provides the clearest voices in a variety of accents.

To produce an approximation of the vizemes that correspond to the phonemes, we have a phoneme-viseme table that we use to produce simple mouth shapes corresponding to the sounds being produced. This is currently an approximation and could use further improvement.

Polly is the recommended option, as it is higher quality, but it
requires setting up an AWS account and enabling polly. Coqui is an open
source synthesizer that runs locally, but the voices are not as high quality.

## Installation

To use polly you will need to [get set up with AWS](https://aws.amazon.com/polly/getting-started/).
To use coqui TTS you will just need to pip install TTS, through more information can be found on
[coqui's github](https://github.com/coqui-ai/TTS)


## Getting started
To generate speech it is easiest to use the speaker module.

To run this file directly, do so from the utils directory:
``` bash
    cd backend/app/utils/
    python -m tts.speaker --savepath=./tts/backends/output/temp.wav
```

### Backend Components
The backend components (polly, coqui tts) are available as self contained modules for individual usage as well.

## Details
For Coqui, we are using the /en/vctk/vits model, as it has a wide range of voices with reasonable acccuracy.
Other models are supported and can be found through their github, including instructions on how to train 
your own voice encoder.

Unfortunately a bug in their training process means the model speaker labels don't match the training set, but we have list of a few speakers that we have matched to the model input names.

## Roadmap
<!-- --8<-- [start:ttsroadmap] -->
The current TTS module is stable, particularly with polly. Coqui and Viseme generation
will need a fair amount of work, but that should not change the fundamental module api.

- [ ] Improved viseme processing
    - This will bee done in conjunction with the frontend, to improve the accuracy and number
    of visemes provided. Additional work needs to be done to improve the timing of the
    custom viseme generation.
<!-- --8<-- [end:ttsroadmap] -->
