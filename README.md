

![Visual of Face](frontend/public/face.png?raw=true "HCI-FACE")

# HCI-FACE
## Human-Computer Interaction Facial Animation and Conversation Engine

_Also Known As: OSCAR, your Open Source Communication AnimatoR_

HCI-FACE is an open source, fully customizable, full stack, interaction engine. It combines a React front end with a python back end to power an animated face capable of real time voice based interaction.

HCI-FACE utilizes freely available, open source elements to power all the interaction components, but also supports easily swapping modules (TTS, STT, chatbot, etc.) for use with paid cloud services.

---
## Getting Started

### Front End

To get started you will need to install the front end:

```npm install```

Then to run the front end:

```npm start```

And the browser should automattically start.

### Back End

To use the back end, install the requirements.txt with your favorite package manager. I suggest [anaconda](https://www.anaconda.com/)  

```
conda create --name hci-face python=3.9

conda activate hci-face

cd backend

pip install -r requirements.txt
```

This may take a while, depending on your internet connection.  

Any huggingface models you choose to use should be installed automatically on first use.

If you wish to use chatGPT (enabled by default) you will need to add your API key to your environment


---
## Components

### Front End

The front end is a single react web page with the following elements:

- An animated face of SVG elements animated with the react-spring library  
- Microphone input passed through a websocket to the python backend  
- A form input for testing and controlling elements of the face  

The face design is easily modified and customized, while utilizing FACs for animation control to maintain animation accross differing visual renderings.

### Back End

The back end consists of python libraries for TTS, STT, and chatbot interaction. The back end is connected to the front end with fastapi. The open source backend components are:

- Text-To-Speech (TTS): TTS uses models powered by the coqui tts library.  
- Speech-To-Text (STT): STT uses whisper models from huggingface.
- Chatbot: The chatbot functionality is currently provided by generative models from huggingface such as GPT-NEO.

These modules are imported into the app/api.py and can be replaced with private/paid modules by switching the imports.

---
## Helpful Resources

### Front End

[Facial Action Units Resources](https://imotions.com/blog/learning/research-fundamentals/facial-action-coding-system/)  

[SVG Drawing Tool](https://svg-path-visualizer.netlify.app/#M%20-28%20-14%20A%208%208%200%201%200%20-8%20-14%20A%208%200%200%201%201%20-28%20-14)

[Emotion Expression](http://www.erasmatazz.com/library/design-diaries/design-diary-siboot/september-2014/moods-and-facial-expression.html)

[Viseme Cheat Sheet](https://melindaozel.com/viseme-cheat-sheet/) and [Lip Synchronization](https://wolfpaulus.com/lipsynchronization/)

### Back End

[Coqui-ai](https://github.com/coqui-ai/TTS)

[Whisper](https://huggingface.co/openai/whisper-large)

[HuggingFace](https://huggingface.co/)

[OpenAI](https://openai.com/)

[Amazon](https://aws.amazon.com/)


---
## TODO List

"*" Items open for contribution


### Front End 

- [ ] Pre-Process audio for speech segments on client side ***
- [ ] Different visuals presets  
    - [ ] Additional idle behaviors
        - Breathing, Thinking
- [ ] organize consistent styles and colors *
- [ ] Create additional face visual presents *
    - Such as QT default, cordial replica, and human mask
- [ ] Improve accuracy of lip synch *
- [ ] Export faces from AU for custom expressions
- [ ] Seperate left and right control

### Back End
- [ ] Test ChatGPT for classification
- [ ] Multi-Speaker input 
- [ ] [Add Amazon Polly Support](https://github.com/awsdocs/aws-doc-sdk-examples/blob/main/python/example_code/polly/polly_wrapper.py)
- [ ] finish face control api (AU control) 
- [ ] Long term interaction recording


### Long Term Goals
- [ ] Interaction Lab greeter
- [ ] meeting note taker  
- [ ] Timer work encouragement  
- [ ] Philosophy Teacher
- [ ] Meditation Leader
- [ ] Basic Q&A
- [ ] Deploy backend to the cloud

### General
- [ ] Improve Documentation (start a read the docs)
    - [ ] How to Customizing the bot appearance * 
    - [ ] How to contribute
    - [ ] How to customize responses
    - [ ] How to add a new API service

