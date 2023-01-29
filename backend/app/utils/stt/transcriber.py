"""Uses the Whisper Model from OpenAI to transcribe audio

The whisper model provides state of the art results at a passable speed.
"""

import os
import tempfile
import csv
import argparse

from pyannote.core.segment import Segment
from .backends import record_mic, WhisperSTT, PyannoteDiarize


class Transcriber:
    """ Helper class which processes audio clips or files

    """

    def __init__(self, model_size: str = "medium", save_dir: str = None) -> None:

        if not save_dir:
            save_dir = tempfile.mkdtemp()
        self.save_dir = save_dir
        self.stt = WhisperSTT(model_size=model_size, save_dir=save_dir)
        self.diarizer = PyannoteDiarize(save_dir=save_dir)

    def transcribe_clip(self, audio_clip):
        """Transcribe bytes of an audio clip"""
        text_transcribed = self.stt.transcribe_clip(audio_clip)
        return text_transcribed

    def transcribe_file(self, file_name, diarize=False):
        """Transcribe a file"""
        transcription_obj = self.stt.transcribe_file(
            file_name=file_name)

        if not diarize:
            return transcription_obj["text"]
        diarization_segments = self.diarize_file(file_name=file_name)

        diarized_speech = self.combine_whisper_annotation(
            transcription_obj["segments"], diarization_segments)

        final_diarization = self.combine_speaker_segments(diarized_speech)
        final_save_path = os.path.join(self.save_dir, "diarized_transcription.csv")
        self.save_csv(final_diarization, final_save_path)
        return final_diarization
            

    def diarize_file(self, file_name=None):
        """Diarize a file or the file that has been saved"""
        diarization_segments = self.diarizer.diarize_file(file_path=file_name)
        return diarization_segments

    def combine_whisper_annotation(self, whisper_segments, diarization_segments):
        """Combines the speaker labels from the diarization with the whisper segments"""
        diarized_speech = []
        outstr = ""

        # Determine which speaker segments overlap with transcription segments
        for whisper_segment in whisper_segments:
            whisp = Segment(whisper_segment["start"], whisper_segment["end"])
            speaker_id = {
                "id": "SPEAKER_00",
                "duration": 0
            }

            for diar_segment in diarization_segments:
                diar, speaker = diar_segment.values()
                intersection = diar & whisp
                duration = intersection.duration
                if duration > speaker_id["duration"]:
                    speaker_id["id"] = speaker
                    speaker_id["duration"] = duration
            speaker_key = speaker_id["id"]

            segment_dict = {
                "text": whisper_segment["text"],
                "start": round(whisper_segment["start"], 2),
                "end": round(whisper_segment["end"], 2),
                "speaker": speaker_key
            }
            diarized_speech.append(segment_dict)
            outstr += f"{whisper_segment['text']} - {speaker_key}  \n"
        return diarized_speech

    def combine_speaker_segments(self, diarized_speech):
        """Combines adjacent segments with the same speaker

        Builds up a new list while iterating through the old segments
        """

        combined_segments = []
        j = 0
        for ind, segment in enumerate(diarized_speech):
            if ind < j:
                continue
            cur_speaker = segment["speaker"]
            start = segment["start"]
            text = ""

            while j < len(diarized_speech) and cur_speaker == diarized_speech[j]["speaker"]:
                text += diarized_speech[j]["text"]
                j += 1

            end = diarized_speech[j-1]["end"]
            new_d = {
                "speaker": cur_speaker,
                "text": text,
                "start": start,
                "end": end,
            }
            combined_segments.append(new_d)

        return combined_segments

    def save_csv(self, diarization, filename="diarization.csv"):
        """Save diarization to csv"""
        with open(filename, 'w') as diraization_file:
            writer = csv.writer(diraization_file)
            writer.writerow(diarization[0].keys())
            for segment in diarization:
                writer.writerow(segment.values())
            diraization_file.close()


def main():
    """Test Transcriber
    
    To run this file directly, do so from the utils directory:
    python -m stt.transcriber
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--model", default="base", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--record", default=False,
                        help="record a 20 second example", type=bool)
    parser.add_argument("--filename", default="stt/backends/output/diarization_test.wav",
                        help="location of file to transcribe")

    args = parser.parse_args()

    transcribe = Transcriber(save_dir="stt/backends/output")
    print("loading complete")

    if args.record:
        record_mic(args.filename)

    result = transcribe.transcribe_file(file_name=args.filename, diarize=True)
    print(result)


if __name__ == "__main__":
    main()
