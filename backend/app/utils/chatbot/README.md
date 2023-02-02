# Chatbot

The interface to chatbot functionality is through the responder module.
We currently include huggingface models and an interface for ChatGPT.
Both can be tested by directly running the zero_shot and chatgpt files.


## Installation
You can follow the huggingface installation instructions and the OpenAI
python api instructions to set up your chatbot.


## Getting Started
There are three basic ways to interact with the chatbot.

1. You can interact directly with the zero_shot or chatgpt modules.
    These can be run directly from the command line.

2. You can strike up an interactive conversation with the responder module
    either through the command line or by importing it into a different script.

3. For more complex projects I recommend looking at the facilitator module to see 
    how the chatbot backend was incorporated for a combination of controlled and
    uncontrolled responses.


## Details


## Roadmap

<!-- --8<-- [start:chatbotroadmap] -->
- [ ] Additional local backends
    - currently just have support for hugginface models and pipeline, will add support for custom pytorch
    or other ml models
- [ ] Additional cloud backends
    - currently just support OpenAI, but in future will add support for other classification and chatbot
    services, starting with aws.
<!-- --8<-- [end:chatbotroadmap] -->
