#!/usr/bin/env python3
"""Utility module for creating audio and visemes from text.

There are two supported options for generating speech, Coqui and Polly.
You must choose between the two with the backend parameter at load time.
Both support a wide variety of voice accents/genders/ages that can be
chosen at runtime by selecting the speaker_identifier.

"""

import argparse
from .backends import PollySpeak, CoquiSpeak, VisemeGenerator


class Speaker:
    """Speaker takes text and returns an audio stream and a viseme list with timing

        Speaker backend is set at runtime, but speaker ID can change dynammically

        Args:
            backend (str, optional): Which backend to use. Defaults to "polly"."""
    def __init__(self, backend="polly") -> None:
        self.backend = backend
        if self.backend == "polly":
            self.speaker = PollySpeak()
        if self.backend == "coqui":
            self.speaker = CoquiSpeak()
        self.viseme_generator = VisemeGenerator()

    def synthesize(self, input_text: str, speaker_identifier: str,
                   save_path: str = "./tts/backends/output/temp.wav") -> tuple:
        """Takes in text and a speaker id and returns speech and visemes and timings

            Args:
                input_text (str): input text to say
                speaker_identifier (str): key for speaker voice
                save_path (str, optional): path to save wav file. Defaults to "./tts/backends/output/temp.wav".

            Returns:
                tuple: the audio stream, the visemes, and the viseme timings"""
        if self.backend == "polly":
            results = self.speaker.synthesize(input_text,
                                              speaker_id=speaker_identifier,
                                              save_path=save_path)
            audio_stream, visemes, delays = results

            visemes = self.viseme_generator.convert_aws_visemes(visemes)

        if self.backend == "coqui":
            results = self.speaker.synthesize_wav(input_text,
                                                  speaker_id=speaker_identifier)
            audio_stream, speaking_time = results

            # Visemes must be generated and timed manually for coqui
            visemes = self.viseme_generator.get_visemes(input_text)
            viseme_length = (speaking_time) / (len(visemes)+1)
            delays = [viseme_length for i in range(len(visemes))]

        return audio_stream, visemes, delays


def main():
    """Run speaker on a text string

    To hear the sound play the file at the save path "./tts/backends/output/temp.wav"
    """
    parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--backend", default="polly", help="Backend to use",
                        choices=["polly", "coqui"])
    parser.add_argument("--text", default="This is what I sound like",
                        help="Text to say")
    parser.add_argument("--speakerid", default="Kevin",
                        help="location of file to transcribe")
    parser.add_argument("--savepath", default="./tts/backends/output/temp.wav",
                    help="location of file to save audio")

    args = parser.parse_args()
    speaker = Speaker(backend=args.backend)
    _, visemes, delays = speaker.synthesize(args.text, args.speakerid, save_path=args.savepath)
    visemes_delays = zip(visemes,delays)
    for i in visemes_delays:
        print(i)

if __name__ == "__main__":
    main()
