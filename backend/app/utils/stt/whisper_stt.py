import whisper
import os, io
import tempfile
import csv
import pyaudio
import wave
import argparse
from pyannote.audio import Pipeline
from pyannote.core.segment import Segment


def record_mic(filename, seconds=200):
    chunk = 1024  # Record in chunks of 1024 samples
    sample_format = pyaudio.paInt16  # 16 bits per sample
    channels = 2
    fs = 16000  # Record at 44100 samples per second
    seconds = 200
    p = pyaudio.PyAudio()  # Create an interface to PortAudio

    print('Recording')

    stream = p.open(format=sample_format,
                    channels=channels,
                    rate=fs,
                    frames_per_buffer=chunk,
                    input=True)

    frames = []  # Initialize array to store frames

    # Store data in chunks for 3 seconds
    for i in range(0, int(fs / chunk * seconds)):
        data = stream.read(chunk)
        frames.append(data)

    # Stop and close the stream 
    stream.stop_stream()
    stream.close()
    # Terminate the PortAudio interface
    p.terminate()

    print('Finished recording')

    # Save the recorded data as a WAV file
    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(sample_format))
    wf.setframerate(fs)
    wf.writeframes(b''.join(frames))
    wf.close()

class Transcriber():
    """ 
    Helper class which processes audio clips or files
    """
    def __init__(self, model_size: str="medium") -> None:

        self.save_path = os.path.join(tempfile.mkdtemp(), "temp.wav")
        self.diarization_pipeline = Pipeline.from_pretrained("pyannote/speaker-diarization", use_auth_token="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")
        self.audio_model = whisper.load_model(model_size)
   
    def transcribe_clip(self, audio_clip):
        audio_clip.export(self.save_path, format="wav")
        result = self.audio_model.transcribe(self.save_path, language='english')
        return result["text"]

    def transcribe_file(self, file_name = None, diarize = False):
        if file_name:
            self.save_path = file_name
        else: print("no filename here")

        result = self.audio_model.transcribe(self.save_path, language='english')
        self.save_csv(result["segments"], "transcription.csv")
        # for s in result["segments"]:
        #     print(s)
        if diarize:
            diarized_segments = self.diarize_segments(result["segments"])
            self.save_csv(diarized_segments, "diarization.csv")
            return diarized_segments
        return result

    def diarize_segments(self, whisper_segments):
        diarization = self.diarization_pipeline(self.save_path)
        diarized_speech = []
        outstr = ""

        # Determine which speaker segments overlap with transcription segments
        for whisper_segment in whisper_segments:
            ws = Segment(whisper_segment["start"],whisper_segment["end"])
            speaker_id = {
                "id":"SPEAKER_00",
                "duration":0
            }

            for segment, _, speaker in diarization.itertracks(yield_label=True):
                intersection = segment & ws
                duration = intersection.duration
                if duration > speaker_id["duration"]:
                    speaker_id["id"] = speaker
                    speaker_id["duration"] = duration
            speaker_key = speaker_id["id"]
            
            segment_dict = {
                "text" : whisper_segment["text"],
                "start" : round(whisper_segment["start"],2),
                "end" : round(whisper_segment["end"],2),
                "speaker" : speaker_key
            }
            diarized_speech.append(segment_dict)
            outstr += f"{whisper_segment['text']} - {speaker_key}  \n"
        
        print("PART2")
        # Join together segments with the same speaker
        diarized_segments = []
        j = 0
        for i in range(len(diarized_speech)):
            if i<j: continue
            cur_speaker = diarized_speech[i]["speaker"]
            start = diarized_speech[i]["start"]
            text = ""
            while j<len(diarized_speech) and cur_speaker == diarized_speech[j]["speaker"]:
                text += diarized_speech[j]["text"]
                j += 1
        
            end = diarized_speech[j-1]["end"]
            new_d = {
                "speaker": cur_speaker,
                "text": text,
                "start": start,
                "end": end,
            }
            diarized_segments.append(new_d)
        for l in diarized_speech:
            print(l)

        return diarized_segments


    def save_csv(self, diarization, filename="diarization.csv"):
        myFile = open(filename,'w')
        writer = csv.writer(myFile)
        writer.writerow(diarization[0].keys())
        for d in diarization:
            writer.writerow(d.values())
        myFile.close()
 

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--model", default="base", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--record", default=False, 
                        help="record a 20 second example", type=bool)
    parser.add_argument("--filename", default="test_input.wav", 
                        help="location of file to transcribe")
    
    args = parser.parse_args()

    tr = Transcriber(model_size = args.model)
    print("loading complete")

    filename = args.filename

    if args.record:
        record_mic(filename)
    
    result = tr.transcribe_file(file_name = filename, diarize=True)


if __name__ == "__main__":
    main() 

