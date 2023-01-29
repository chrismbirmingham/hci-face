""" Speaker wraps tts backends and exposes common API for app

"""

from .backends import PollySpeak, CoquiSpeak, VisemeGenerator


class Speaker:
    """Speaker takes text and returns an audio stream and a viseme list with timing
    
    Speaker backend is set at runtime, but speaker ID can change dynammically
    """
    def __init__(self, backend="polly") -> None:
        self.backend = backend
        if self.backend == "polly":
            self.speaker = PollySpeak()
        if self.backend == "coqui":
            self.speaker = CoquiSpeak()
        self.viseme_generator = VisemeGenerator()
        
    def synthesize(self, input_text: str, speaker_identifier: str):
        """Takes in text and a speaker id and returns speech and visemes and timings"""
        if self.backend == "polly":
            audio_stream, visemes, delays = self.speaker.synthesize(input_text,speaker_id=speaker_identifier)
            visemes = self.viseme_generator.convert_aws_visemes(visemes)

        if self.backend == "coqui":
            audio_stream, speaking_time = self.speaker.synthesize_wav(input_text, speaker_id=speaker_identifier)
            visemes = self.viseme_generator.get_visemes(input_text)
            viseme_length = (speaking_time) / (len(visemes)+1)
            delays = [viseme_length for i in range(len(visemes))]

        return audio_stream, visemes, delays


def main():
    """Integration testing for speaker"""
    speaker = Speaker(backend="polly")
    audio_stream, visemes, delays = speaker.synthesize("This is a test", "Kevin")
    z = zip(visemes,delays)
    for i in z:
        print(i)

    speaker = Speaker(backend="coqui")
    audio_stream, visemes, delays = speaker.synthesize("This is a test", "p267")
    z = zip(visemes,delays)
    for i in z:
        print(i)

if __name__ == "__main__":
    main()
