#!/usr/bin/env python3
"""Module for calling on AWS Polly

warning: Requires AWS setup for polly.
    Create a client using the credentials and region defined in the [adminuser]
    section of the AWS credentials file (~/.aws/credentials).

Will play the speech live.
"""

import io
import os
import sys
import json
import subprocess
import time
from pathlib import Path
from contextlib import closing
from boto3 import Session
from botocore.exceptions import BotoCoreError, ClientError
try:
    from viseme_generator import VisemeGenerator
    from audio_viseme_player import play_audio_viseme
except ImportError:
    from .audio_viseme_player import play_audio_viseme
    from .viseme_generator import VisemeGenerator

session = Session(profile_name="default")
polly = session.client("polly")

english_speaker_map = {
    "en-US": ['Kevin', 'Salli', 'Matthew', 'Kimberly', 'Kendra', 'Justin', 'Joey', 'Joanna', 'Ivy'],
    "en-NZ": ['Aria'],
    "en-ZA": ['Ayanda'],
    "en-GB": ['Emma', 'Brian', 'Amy', 'Arthur'],
    "en-AU": ['Olivia'],
    "en-IN": ['Kajal'],
}


class PollySpeak():
    """ Synthesizes speech with AWS Polly

    Possible english speakers include:
        US English en-US {'Kevin', 'Salli', 'Matthew', 'Kimberly', 'Kendra',
                            'Justin', 'Joey', 'Joanna', 'Ivy'}
        New Zealand English en-NZ {'Aria'}
        South African English en-ZA {'Ayanda'}
        British English en-GB {'Emma', 'Brian', 'Amy', 'Arthur'}
        Australian English en-AU {'Olivia'}
        Indian English en-IN {'Kajal'}
    """

    def __init__(self, default_path: str = "./output/temp.wav") -> None:
        self.engine = "neural"
        self.audio_format = "mp3"
        self.polly_client = polly
        self.path = Path(__file__).parent
        self.save_path = os.path.join(self.path, default_path)
        self.viseme_generator = VisemeGenerator()

    def synthesize(self, text: str, speaker_id: str = "", save_path: str = None):
        """Turns text into audio and visemes"""
        if save_path:
            self.save_path = save_path
        lang_code = None
        for key, names in english_speaker_map.items():
            if speaker_id in names:
                lang_code = key
                voice = speaker_id
        if not lang_code:
            lang_code = "en-US"
            voice = 'Kendra'
        try:
            kwargs = {
                'Engine': self.engine,
                'OutputFormat': self.audio_format,
                'Text': text,
                'VoiceId': voice}
            if lang_code is not None:
                kwargs['LanguageCode'] = lang_code

            response = self.polly_client.synthesize_speech(**kwargs)
            audio_stream = response['AudioStream']
            output = self.save_path
            with closing(audio_stream) as stream:
                with open(output, "wb") as file:
                    file.write(stream.read())
            outstream = io.open(output, 'rb', buffering=0)
            visemes = None
            kwargs['OutputFormat'] = 'json'
            kwargs['SpeechMarkTypes'] = ['viseme']
            response = self.polly_client.synthesize_speech(**kwargs)
            visemes = [json.loads(viseme) for viseme in
                       response['AudioStream'].read().decode().split() if viseme]
            viseme_list = []
            time_list = []
            for viseme in visemes:
                viseme_list.append(viseme["value"])
                time_list.append(viseme["time"])
            sleep_times = []
            t_before = 0
            for next_t in time_list:
                wait_seconds = float(next_t) - float(t_before)
                sleep_times.append(wait_seconds/1000)
                t_before = next_t
        except ClientError as exc:
            print(exc)
            raise
        else:
            visemes = self.viseme_generator.convert_aws_visemes(viseme_list)
            return outstream, visemes, sleep_times


def main():
    """Testing the integrated functionality of PollySpeak"""
    default_path="./output/temp.mp3"
    speaker = PollySpeak(default_path=default_path)
    example = ("Please call Stella.  Ask her to bring these things with her from the store: "
               "Six spoons of fresh snow peas, five thick slabs of blue cheese, "
               "and maybe a snack for her brother Bob.  We also need a small plastic snake "
               "and a big toy frog for the kids.  She can scoop these things into three red bags, "
               "and we will go meet her Wednesday at the train station.")

    try:
        _, visemes, delays = speaker.synthesize(example, "Aria")
    except (BotoCoreError, ClientError) as error:
        print(error)
        sys.exit(-1)

    play_audio_viseme(default_path, visemes, delays)


if __name__ == "__main__":
    main()
