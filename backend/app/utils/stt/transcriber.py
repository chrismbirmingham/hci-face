#!/usr/bin/env python3
"""Utility module for transcribing audio clips or files.

It uses the SpeechRecognition package for transcription of short audio clips
and the pyannote model for diarization. Currently the diarization is not
integrated with the front end API, but this is on the roadmap.

Note: What is real time?
    The STT module originally included methods for processing incoming audio
    streams that were sent over a websocket to produce live transcription
    results. This was abandonded in favor of having the audio processing
    occur on the front end for two reasons. One, setting up websockets for
    remote connections is unecessarily complicated. Two, placing the websocket
    in the frontend easily exposes parameters for controlling speech sensitivity
    which will vary with environment and microphone.
"""

import os
import tempfile
import argparse
from pydub import AudioSegment
import speech_recognition as sr


class Transcriber:
    """Wraps the speech_recognition module

        Args:
            model_size (str, optional): Size of the whisper model to use. Defaults to "medium".
            save_dir (str, optional): Location to save the transcribed audio. Defaults to None.

        Attributes:
            stt (obj): Exposes transcribe_clip and transcribe_file method
            common_hallucinations (list[str]): List of common hallucinations
                produced by the whisper stt model."""
    def __init__(self, service: str = "google", model_size: str = "small", save_dir: str = None) -> None:

        if service not in ["google", "whisper"]:
            raise ValueError(f"Service {service} not supported")

        if service == "google" and not os.environ.get("GOOGLE_APPLICATION_CREDENTIALS"):
            print("set GOOGLE_APPLICATION_CREDENTIALS to the path of your google cloud credentials json file")
            # export GOOGLE_APPLICATION_CREDENTIALS="/path/to/credentials.json"
            raise ValueError("Google service requires credentials to be set in the environment variable GOOGLE_APPLICATION_CREDENTIALS")

        if service == "google":
            print("Using Google Speech Recognition, model size is ignored")

        self.common_hallucinations = ["        you","       You",
            "          Thanks for watching!","  Thank you for watching!",
            "        THANK YOU FOR WATCHING!","   THANKS FOR WATCHING!",
            "Thanks for watching! Don't forget to like, comment and subscribe!"]

        self.service = service
        self.stt = sr.Recognizer()
        self.model_size = model_size

        if not save_dir:
            save_dir = tempfile.mkdtemp()
        self.save_dir = save_dir

    def _save_audio(self, audio_clip: AudioSegment, file_name: str) -> None:
        """Save audio clip to file

            Args:
                audio_clip (AudioSegment): bytes read from a file containing speech
                file_name (str): Path to save audio clip to."""
        with open(file_name, "wb") as f:
            f.write(audio_clip.get_wav_data())
    
    def transcribe_clip(self, audio_clip: AudioSegment) -> str:
        """Transcribes audio segment

            Args:
                audio_clip (AudioSegment): bytes read from a file containing speech

            Returns:
                str: the transcribed text. returns "" if the audio was a hallucination"""
            
        text_transcribed = ""
        try:
            if self.service == "google":
                # text_transcribed = self.stt.recognize_google(audio_clip)
                # for testing purposes, we can just using the default API key
                # to use another API key, use
                # `self.stt.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                # instead of `self.stt.recognize_google(audio)` but for real use set up the cloud properly:
                text_transcribed = self.stt.recognize_google_cloud(audio_clip, credentials_json=os.environ["GOOGLE_APPLICATION_CREDENTIALS"])
                text_transcribed = text_transcribed[0].upper() + text_transcribed[1:-1] + "."
            elif self.service == "whisper":
                text_transcribed = self.stt.recognize_whisper(audio_clip,model=self.model_size, language="english")
            else:
                raise ValueError(f"Service {self.service} not supported")
                
        except sr.UnknownValueError:
            print(f"{self.service} could not understand audio")
        except sr.RequestError as error:
            print(f"Could not request results from {self.service} service; {error}")


        for option in self.common_hallucinations:
            if text_transcribed in option:
                return ""
        self._save_audio(audio_clip, os.path.join(self.save_dir, "transcribed.wav"))
        return text_transcribed

    def transcribe_file(self, file_name: str) -> str:
        """Transcribe a file
        
            Transcribes a complete audio file
            Args:
                file_name (str): Path to audio file.

            Returns:
                str: transcription of text from audio."""
        if file_name[-3:] == "mp3":
            audio = AudioSegment.from_file(file_name)
            file_name = os.path.join(self.save_dir,"temp.wav")
            audio.export(file_name, format="wav")

        with sr.AudioFile(file_name) as source:
            audio = self.stt.record(source)  # read the entire audio file
            text_transcribed = self.transcribe_clip(audio)

        return text_transcribed

    def live_transcribe(self) -> None:
        """Transcribe audio from a microphone.

            Args:
                service (str, optional): Service to use. Defaults to "google".
        """
        with sr.Microphone() as source:
            print("getting ambient noise...")
            self.stt.adjust_for_ambient_noise(source)  # listen for 1 second to calibrate
            result = ""
            while "stop" not in result:
                print("Say something! I will listen till you pause...")
                audio = self.stt.listen(source)
                print("Recognizing Now...")
                result = self.transcribe_clip(audio)
                print(result)




def main():
    """Run transcriber on a wav file."""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--service", default="google", help="Service to use",
                        choices=["google", "whisper"])
    parser.add_argument("--model", default="base", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--record", default=False,
                        help="record from the microphone to transcribe until 'stop'", type=bool)
    parser.add_argument("--filename", default="output/test.wav",
                        help="location of file to transcribe")
    parser.add_argument("--savedir", default=None,
                    help="location of file to save transcription")

    args = parser.parse_args()


    if args.record:
        transcribe = Transcriber(save_dir=args.savedir, service=args.service)
        transcribe.live_transcribe()

    else:

        for m in ["tiny", "base", "small", "medium", "large"]:
            transcribe = Transcriber(service="whisper", model_size=m)
            print(f"loading {m} complete")

            transcriptions = transcribe.transcribe_file(file_name=args.filename)
            print(transcriptions)

        transcribe = Transcriber(service="google")
        transcriptions = transcribe.transcribe_file(file_name=args.filename)
        print(transcriptions)

        
if __name__ == "__main__":
    main()
