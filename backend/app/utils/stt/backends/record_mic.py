"""Helper function for recording a microphone to a file"""

import argparse
import wave
import pyaudio


def record_mic(filename, seconds=200):
    """Helper function for recording a microphone to a file"""
    chunk = 1024  # Record in chunks of 1024 samples
    sample_format = pyaudio.paInt16  # 16 bits per sample
    channels = 2
    framerate = 16000  # Record at 44100 samples per second
    audio_interface = pyaudio.PyAudio()  # Create an interface to PortAudio

    print('Recording')

    stream = audio_interface.open(format=sample_format,
                    channels=channels,
                    rate=framerate,
                    frames_per_buffer=chunk,
                    input=True)

    frames = []  # Initialize array to store frames

    # Store data in chunks for 3 seconds
    for _ in range(0, int(framerate / chunk * seconds)):
        data = stream.read(chunk)
        frames.append(data)

    # Stop and close the stream
    stream.stop_stream()
    stream.close()
    # Terminate the PortAudio interface
    audio_interface.terminate()

    print('Finished recording')

    # Save the recorded data as a WAV file
    wavefile = wave.open(filename, 'wb')
    wavefile.setnchannels(channels)
    wavefile.setsampwidth(audio_interface.get_sample_size(sample_format))
    wavefile.setframerate(framerate)
    wavefile.writeframes(b''.join(frames))
    wavefile.close()


def main():
    """Example of making a simple audio clip"""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--record_for", default=20,
                        help="Set recording length (seconds)", type=bool)
    parser.add_argument("--filename", default="output/test.wav",
                        help="location to save audio file")

    args = parser.parse_args()

    record_mic(args.filename, seconds=args.record_for)
    print(f"File saved to {args.filename}")

if __name__ == "__main__":
    main()
