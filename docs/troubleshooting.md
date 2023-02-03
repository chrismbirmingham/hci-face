# Troubleshooting Help
This is a record of all the errors that have cropped up and how they were fixed.

#### Address already in use
If the address of the mkdocs is already in use, you can change the dev_addr parameter in mkdocs.ym


## Frontend Troubleshooting

#### Permission Denied when running npm start
The npm package may not be installed, particularly if you deleted it or switched branches.

Fix: Reinstall with ```npm install``` before attempting to start again

#### The visemes don't appear to run as long as the audio
This is probably caused by more than one client to the viseme stream. Make sure you have only one face open.

#### No expressions or visemes appear as directed
Make sure the face is requesting messages from your computer's IP. You can find this with ifconfig and update
the server_ip in face/src/app.js

## Backend Troubleshooting

#### I can't seem to install pyaudio
Pyaudio is based on PortAudio which allows for cross system audio.

If you have trouble installing pyaudio, I recommend following (these directions)[https://people.csail.mit.edu/hubert/pyaudio/].

#### pyannote requires an authorization token:
1. visit hf.co/pyannote/speaker-diarization and hf.co/pyannote/segmentation 
  and accept user conditions (only if requested)
2. visit hf.co/settings/tokens to create an access token (only if you had to go through 1.)

#### CUDA & GPU Requirements
Because of cloud supported models, CUDA is optional. CUDA is only required for the zero_shot module in the chatbot utility. For the coqui tts module gpu acceleration is optional and is currently off by default.

If you would like to use HCI-FACE with GPU support and are running into issues with CUDA or NVIDIA drivers, I typically recommend purging your system and installing them from scratch.
