#!/usr/bin/env python3
"""Utility module for transcribing audio clips or files.

It uses the whisper model for real time transcription of short audio clips
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
import csv
import argparse
from pydub import AudioSegment
from pyannote.core.segment import Segment
from .backends import WhisperSTT, PyannoteDiarize
# from .backends import record_mic


class Transcriber:
    """Wraps the whisper module with pyannotate diarization and microphone recording.

        When transcribing a clip it will just return the text, but when transcribing a
        file there is the option to return diarized speech. When diarizing, it will save
        the diarized text to a csv.

        Args:
            model_size (str, optional): Size of the whisper model to use. Defaults to "medium".
            save_dir (str, optional): Location to save the transcribed audio. Defaults to None.

        Attributes:
            stt (obj): Exposes transcribe_clip and transcribe_file method
            diarizer (obj): Exposes diarize_file method
            common_hallucinations (list[str]): List of common hallucinations
                produced by the whisper stt model."""
    def __init__(self, model_size: str = "medium", save_dir: str = None) -> None:
        self.common_hallucinations = ["        you","       You",
            "          Thanks for watching!","  Thank you for watching!",
            "        THANK YOU FOR WATCHING!","   THANKS FOR WATCHING!",
            "Thanks for watching! Don't forget to like, comment and subscribe!"]
        if not save_dir:
            save_dir = tempfile.mkdtemp()
        self.save_dir = save_dir
        self.stt = WhisperSTT(model_size=model_size, save_dir=save_dir)
        self.diarizer = PyannoteDiarize(save_dir=save_dir)

    def transcribe_clip(self, audio_clip: AudioSegment) -> str:
        """Transcribes audio segment

            Args:
                audio_clip (AudioSegment): bytes read from a file containing speech

            Returns:
                str: the transcribed text. returns "" if the audio was a hallucination"""
        text_transcribed = self.stt.transcribe_clip(audio_clip)
        for option in self.common_hallucinations:
            if text_transcribed in option:
                return ""
        return text_transcribed

    def transcribe_file(self, file_name: str, diarize: bool = False) -> str:
        """Transcribe a file, diarize if specified.
        
            Transcribes a complete audio file, will save diarized speech if
            diarize is True.

            Args:
                file_name (str): Path to audio file.
                diarize (bool, optional): Diarize the audio. Defaults to False.

            Returns:
                str: transcription of text from audio."""
        transcription_obj = self.stt.transcribe_file(file_name)

        if not diarize:
            return transcription_obj["text"]
        else:
            diarization_segments = self.diarize_file(file_name=file_name)

            diarized_speech = self._label_transcription(
                transcription_obj["segments"], diarization_segments)

            final_diarization = self._combine_speaker_segments(diarized_speech)
            
            final_save_path = os.path.join(self.save_dir, "diarized_transcription.csv")
            self.save_csv(final_diarization, final_save_path)
            
            return final_diarization

    def diarize_file(self, file_name: str):
        """Wrapper to diarize a file.

            Yes. I know I am probably playing fast and loose with grammar here.

            Args:
                file_name (str): Path to the file.

            Returns:
                list[dict]: list of speaker and time segments."""
        diarizations = self.diarizer.diarize_file(file_path=file_name)
        return diarizations

    def _label_transcription(self, transcription_segments: list, diarization_segments: list) -> list:
        """Combines the speaker labels from the diarization with the transcription segments

            Iterates through transcription segments and finds the best matching diarization
            segment.

            Args:
                transcription_segments (list): transcription with start and end properties.
                diarization_segments (list): diarization segments with speaker labels

            Returns:
                list[dict]: list of segmentation dictionaries"""
        diarized_speech = []
        outstr = ""

        for transcription_segment in transcription_segments:
            whisp_seg = Segment(transcription_segment["start"], transcription_segment["end"])
            speaker_id = {"id": "SPEAKER_00","duration": 0} # default

            for diar_segment in diarization_segments:
                diar_seg, speaker = diar_segment.values()

                # Determine the overlap in segments
                intersection = diar_seg & whisp_seg
                duration = intersection.duration
                if duration > speaker_id["duration"]:
                    speaker_id["id"] = speaker
                    speaker_id["duration"] = duration
                # Keeps the largest overlapping id
            speaker_key = speaker_id["id"]

            segment_dict = {
                "text": transcription_segment["text"],
                "start": round(transcription_segment["start"], 2),
                "end": round(transcription_segment["end"], 2),
                "speaker": speaker_key
            }
            diarized_speech.append(segment_dict)
            outstr += f"{transcription_segment['text']} - {speaker_key}  \n"
        return diarized_speech

    def _combine_speaker_segments(self, diarized_speech: list) -> list:
        """Combines adjacent segments with the same speaker

            Builds up a new list while iterating through the old segments

            Args:
                diarized_speech (list): List of labeled transcription segments

            Returns:
                list: Joined list of labeled transcription segments"""

        combined_segments = []
        counter = 0
        for ind, segment in enumerate(diarized_speech):
            if ind < counter:
                continue

            cur_speaker = segment["speaker"]
            start = segment["start"]
            text = ""

            while counter < len(diarized_speech) and cur_speaker == diarized_speech[counter]["speaker"]:
                text += diarized_speech[counter]["text"]
                counter += 1

            end = diarized_speech[counter-1]["end"]
            new_d = {
                "speaker": cur_speaker,
                "text": text,
                "start": start,
                "end": end,
            }
            combined_segments.append(new_d)

        return combined_segments

    def save_csv(self, diarization: list, filename="diarization.csv") -> None:
        """Save diarization to csv"""
        with open(filename, 'w') as diraization_file:
            writer = csv.writer(diraization_file)
            writer.writerow(diarization[0].keys())
            for segment in diarization:
                writer.writerow(segment.values())
            diraization_file.close()


def main():
    """Run transcriber on a wav file."""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--model", default="base", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--record", default=False,
                        help="record a 20 second example", type=bool)
    parser.add_argument("--filename", default="stt/backends/output/diarization_test.wav",
                        help="location of file to transcribe")
    parser.add_argument("--savedir", default="stt/backends/output/",
                    help="location of file to save diarized transcription")

    args = parser.parse_args()

    transcribe = Transcriber(save_dir=args.savedir)
    print("loading complete")

    if args.record:
        record_mic(args.filename)

    result = transcribe.transcribe_file(file_name=args.filename, diarize=True)
    print(result)


if __name__ == "__main__":
    main()
