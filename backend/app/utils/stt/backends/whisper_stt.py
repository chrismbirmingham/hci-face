"""Uses the Whisper Model from OpenAI to transcribe audio

The whisper model provides state of the art results at a passable speed.
"""

import os
import tempfile
import csv
import argparse
import whisper

class WhisperSTT:
    """ Helper class which processes audio clips or files

    """

    def __init__(self, model_size: str = "medium", save_dir: str = None) -> None:

        if not save_dir:
            save_dir = tempfile.mkdtemp()

        self.default_wave_path = os.path.join(save_dir, "test.wav")
        self.transcription_path = os.path.join(save_dir, "transcription.csv")
        self.audio_model = whisper.load_model(model_size)

    def transcribe_clip(self, audio_clip):
        """Transcribe bytes of an audio clip"""
        audio_clip.export(self.default_wave_path, format="wav")
        result = self.audio_model.transcribe(self.default_wave_path, language='english')
        return result["text"]

    def transcribe_file(self, file_name=None):
        """Transcribe a file"""
        if file_name:
            self.default_wave_path = file_name

        result = self.audio_model.transcribe(self.default_wave_path, language='english')
        self.save_csv(result["segments"], self.transcription_path)
        return result

    def save_csv(self, segments, filename="speech_segments.csv"):
        """Save csv of speech segments"""
        with open(filename, 'w') as my_file:
            writer = csv.writer(my_file)
            writer.writerow(segments[0].keys())
            for seg in segments:
                writer.writerow(seg.values())
            my_file.close()


def main():
    """Test of WhisperSTT"""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--model", default="base", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--filename", default="output/test.wav",
                        help="location of file to transcribe")

    args = parser.parse_args()

    transcriber = WhisperSTT(model_size=args.model, save_dir="output")
    print("loading complete")

    filename = args.filename

    result = transcriber.transcribe_file(file_name=filename)
    print(result['text'])


if __name__ == "__main__":
    main()
