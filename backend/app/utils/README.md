## HCI-FACE Utility Module Logic

The basic components of a control loop are Sense-Plan-Act (repeat). For a conversational agent, we map the basic control loop as follows:

Sense -> Takes human speech and turns it into text (STT)

Plan -> The chatbot must then decide how to respond (Chatbot)

Act -> The robot response must be converted back into speech, including audio and visemes (TTS)

For each of these modules (STT, Chatbot, TTS), we would like to support options for local and cloud based methods, or backends. These backends must have a common interface to expose to the API, so we have wrapped the backend module with a common interface where the backend can be chosen at run time.

We plan to add more supported backends for each module as soon as possible.

