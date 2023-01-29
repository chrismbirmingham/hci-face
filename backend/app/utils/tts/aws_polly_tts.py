from boto3 import Session
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import os
import sys
import json
import subprocess
from tempfile import gettempdir
import time

# Create a client using the credentials and region defined in the [adminuser]
# section of the AWS credentials file (~/.aws/credentials).
session = Session(profile_name="default")
polly = session.client("polly")

english_speaker_map = {
    "en-US" :['Kevin', 'Salli', 'Matthew', 'Kimberly', 'Kendra', 'Justin', 'Joey', 'Joanna', 'Ivy'],
    "en-NZ" :['Aria'],
    "en-ZA" :['Ayanda'],
    "en-GB" :['Emma', 'Brian', 'Amy', 'Arthur'],
    "en-AU" :['Olivia'],
    "en-IN" :['Kajal'],
}

class PollySpeak():
    """
    US English en-US {'Kevin', 'Salli', 'Matthew', 'Kimberly', 'Kendra', 'Justin', 'Joey', 'Joanna', 'Ivy'}
    New Zealand English en-NZ {'Aria'}
    South African English en-ZA {'Ayanda'}
    British English en-GB {'Emma', 'Brian', 'Amy', 'Arthur'}
    Australian English en-AU {'Olivia'}
    Indian English en-IN {'Kajal'}
    """
    def __init__(self) -> None:
        self.engine = "neural"
        self.audio_format = "mp3"
        self.polly_client = polly
        pass

    def synthesize(self, text: str, speaker_id: str = ""):
        for key, names in english_speaker_map.items():
            if speaker_id in names: 
                lang_code=key
                voice=speaker_id
        try:
            kwargs = {
                'Engine': self.engine,
                'OutputFormat': self.audio_format,
                'Text': text,
                'VoiceId': voice}
            if lang_code is not None:
                kwargs['LanguageCode'] = lang_code

            response = self.polly_client.synthesize_speech(**kwargs)
            print("got response", response)
            audio_stream = response['AudioStream']
            output = os.path.join("./output", "speech2.mp3")
            with closing(audio_stream) as stream:
                with open(output, "wb") as file:
                    file.write(stream.read())
            visemes = None
            kwargs['OutputFormat'] = 'json'
            kwargs['SpeechMarkTypes'] = ['viseme']
            response = self.polly_client.synthesize_speech(**kwargs)
            visemes = [json.loads(v) for v in
                        response['AudioStream'].read().decode().split() if v]
            viseme_list = []
            time_list = []
            for v in visemes:
                viseme_list.append(v["value"])
                time_list.append(v["time"])
            sleep_times = []
            t_before = 0
            for i in range(len(time_list)):
                next_t = time_list[i]
                w = float(next_t) - float(t_before)
                sleep_times.append(w/1000)
                t_before = next_t
        except ClientError as e:
            print(e)
            raise
        else:
            return audio_stream, viseme_list, sleep_times





if __name__ == "__main__":
    s = PollySpeak()
    example_text = "Please call Stella.  Ask her to bring these things with her from the store:  Six spoons of fresh snow peas, five thick slabs of blue cheese, and maybe a snack for her brother Bob.  We also need a small plastic snake and a big toy frog for the kids.  She can scoop these things into three red bags, and we will go meet her Wednesday at the train station."

    try:
        # Request speech synthesis
        audio_stream, viseme_list, time_list = s.synthesize(example_text,"Aria")
    except (BotoCoreError, ClientError) as error:
        # The service returned an error, exit gracefully
        print(error)
        sys.exit(-1)

    # Play the audio using the platform's default player
    if sys.platform == "win32":
        os.startfile(output)
    else:
        # The following works on macOS and Linux. (Darwin = mac, xdg-open = linux).
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        output = os.path.join("./output", "speech2.mp3")
        

        subprocess.call([opener, output])
        for i in range(len(time_list)):
            time.sleep(time_list[i])
            print(time_list[i], viseme_list[i])



