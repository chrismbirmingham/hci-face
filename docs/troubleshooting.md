# Troubleshooting Help
This is a record of all the errors that have cropped up and how they were fixed.

#### Address already in use
If the address of the mkdocs is already in use, you can change the dev_addr parameter in mkdocs.yml


## Frontend Troubleshooting

#### Permission Denied when running npm start
The npm package may not be installed, particularly if you deleted it or switched branches.

Fix: Reinstall with ```npm install``` before attempting to start again

## Backend Troubleshooting

#### I can't seem to install pyaudio
Pyaudio is based on PortAudio which allows for cross system audio.

If you have trouble installing pyaudio, I recommend following (these directions)[https://people.csail.mit.edu/hubert/pyaudio/].

#### CUDA
Make sure you have CUDA correctly installed and configured. A non-cuda configuration of HCI-FACE is coming soom.