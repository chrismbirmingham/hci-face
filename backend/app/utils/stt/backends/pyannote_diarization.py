"""Uses the Pyannot Diarization Module

"""

import os
import tempfile
import csv
import argparse
from pyannote.audio import Pipeline


class PyannoteDiarize:
    """ Helper class which processes audio clips or files

    """
    def __init__(self, save_dir: str=None) -> None:

        if not save_dir:
            save_dir=tempfile.mkdtemp()

        self.diarization_path = os.path.join(save_dir, "diarization.csv")
        self.diarization_pipeline = Pipeline.from_pretrained("pyannote/speaker-diarization", use_auth_token="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")
   

    def diarize_file(self, file_path=None):
        """Diarize a file or the file that has been saved"""
        diarization_output = self.diarization_pipeline(file_path)
        self.diarization_segments = []
        for segment, _, speaker in diarization_output.itertracks(yield_label=True):
            self.diarization_segments.append({
                "segment":segment,
                "speaker id":speaker
            })

        self.save_csv(self.diarization_segments, filename=self.diarization_path)
        return self.diarization_segments

    def save_csv(self, diarization, filename="diarization.csv"):
        """Save diarization to csv"""
        with open(filename,'w') as diraization_file:
            writer = csv.writer(diraization_file)
            writer.writerow(diarization[0].keys())
            for segment in diarization:
                writer.writerow(segment.values())
            diraization_file.close()
 

def main():
    """Test PyannoteDiarize"""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--filename", default="output/test.wav",
                        help="location of file to transcribe")
    args = parser.parse_args()

    diarizer = PyannoteDiarize(save_dir="output")

    results = diarizer.diarize_file(file_path=args.filename)
    for segment in results:
        print(segment)



if __name__ == "__main__":
    main()
