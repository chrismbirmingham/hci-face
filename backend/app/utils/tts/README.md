# Text To Speech

The text to speech is done through an open source library courtesy of coqui.ai. This produces a wav file for a given text input. Many models are available through coqui, but we utilize the VITS end-to-end model trained on the VCTK multi-speaker dataset, as this provides the clearest voices in a variety of accents.

Unfortunately a bug in their training process means the model speaker labels don't match the training set, but wwe have list of a few speakers that we have matched to the model input names.

To produce an approximation of the vizemes that correspond to the phonemes, we have a phoneme-viseme table that we use to produce simple mouth shapes corresponding to the sounds being produced. This is currently an approximation and could use further improvement.