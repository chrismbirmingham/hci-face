#!/usr/bin/env python3
""" Open Source Module wrapping coqui's library for text to speech.

This is a wrapper of coqui's TTS library for speech synthesis.
    
note: Punctuation can be used to control the flow of speech.
    period and ?, long pause from breaking into list
    colon, semicolon, comma short pause
    ! breaks things up unpredictably
"""

import os
import io
import sys
import json
import argparse
import subprocess
import time
from pathlib import Path
from typing import Union
import soundfile as sf
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer
try:
    from viseme_generator import VisemeGenerator
    from audio_viseme_player import play_audio_viseme
except ImportError:
    from .audio_viseme_player import play_audio_viseme
    from .viseme_generator import VisemeGenerator

def create_argparser():
    """Parse args to set defaults"""

    def convert_boolean(item):
        return item.lower() in ["true", "1", "yes"]

    parser = argparse.ArgumentParser()
    parser.add_argument("--list_models", type=convert_boolean, nargs="?", const=True, default=False,
        help="list available pre-trained tts and vocoder models.")
    parser.add_argument( "--model_name", type=str, default="tts_models/en/vctk/vits",
        help="Name the pre-trained tts model in format <language>/<dataset>/<model_name>")
    parser.add_argument("--vocoder_name", type=str, default=None,
        help="name of one of the released vocoder models.")
    parser.add_argument("--config_path", default=None,
                        type=str, help="Path to model config file.")
    parser.add_argument( "--model_path", type=str, default=None, help="Path to model file.",)
    parser.add_argument( "--vocoder_path", type=str, default=None,
        help="Path to vocoder model file. If it is not defined, model uses GL as vocoder."
        "Please make sure that you installed vocoder library before (WaveRNN).")
    parser.add_argument("--vocoder_config_path", type=str,
                        help="Path to vocoder model config file.", default=None)
    parser.add_argument("--speakers_file_path", type=str,
                        help="JSON file for multi-speaker model.", default=None)
    parser.add_argument("--port", type=int, default=5002,
                        help="port to listen on.")
    parser.add_argument("--use_cuda", type=convert_boolean,
                        default=False, help="true to use CUDA.")
    parser.add_argument("--debug", type=convert_boolean,
                        default=False, help="true to enable Flask debug mode.")
    parser.add_argument("--show_details", type=convert_boolean,
                        default=False, help="Generate model detail page.")
    return parser


def style_wav_uri_to_dict(style_wav: str) -> Union[str, dict]:
    """Transform an uri style_wav, in either a string (path to wav file to be
    use for style transfer) or a dict (gst tokens/values to be use for styling)

    Args:
        style_wav (str): uri

    Returns:
        Union[str, dict]: path to file (str) or gst style (dict)
    """
    if style_wav:
        if os.path.isfile(style_wav) and style_wav.endswith(".wav"):
            return style_wav  # style_wav is a .wav file located on the server

        style_wav = json.loads(style_wav)
        # style_wav is a gst dictionary with {token1_id : token1_weigth, ...}
        return style_wav
    return None


class CoquiSpeak:
    """Setup Model and perform synthesis

    Speakers of note for the vits model:
        267*,307 - English male, medium
        330*,232 - English male, slow
        312*,251 - English male, fast
        287,254 - English male, fast and deep
        303 - English female, slow
        306 - English female, medium
        308 - English female, slow
        295*,270 - American female, slow
        317* - American male, slow
        230* - American male, fast
        345 - south african female, slow
        313,233 - ? male, fast
        * preferred speakers
    """

    def __init__(self) -> None:
        args, _ = create_argparser().parse_known_args()

        self.path = Path(__file__).parent
        self.save_path = ""

        manager = ModelManager(os.path.join(
            self.path, "resources/.coqui_tts_models.json"))

        # update in-use models to the specified released models.
        model_path = None
        config_path = None
        speakers_file_path = None
        vocoder_path = None
        vocoder_config_path = None

        # CASE1: list pre-trained TTS models
        if args.list_models:
            manager.list_models()
            sys.exit()

        # CASE2: load pre-trained model paths
        if args.model_name is not None and not args.model_path:
            model_path, config_path, model_item = manager.download_model(
                args.model_name)
            args.vocoder_name = model_item["default_vocoder"] if args.vocoder_name is None else args.vocoder_name

        if args.vocoder_name is not None and not args.vocoder_path:
            vocoder_path, vocoder_config_path, _ = manager.download_model(
                args.vocoder_name)

        # CASE3: set custom model paths
        if args.model_path is not None:
            model_path = args.model_path
            config_path = args.config_path
            speakers_file_path = args.speakers_file_path

        if args.vocoder_path is not None:
            vocoder_path = args.vocoder_path
            vocoder_config_path = args.vocoder_config_path

        # load models
        self.synth = Synthesizer(
            tts_checkpoint=model_path,
            tts_config_path=config_path,
            tts_speakers_file=speakers_file_path,
            tts_languages_file=None,
            vocoder_checkpoint=vocoder_path,
            vocoder_config=vocoder_config_path,
            encoder_checkpoint="",
            encoder_config="",
            use_cuda=args.use_cuda,
        )

        self.use_multi_speaker = hasattr(self.synth.tts_model, "num_speakers") and (
            self.synth.tts_model.num_speakers > 1 or self.synth.tts_speakers_file is not None
        )

        self.speaker_manager = getattr(
            self.synth.tts_model, "speaker_manager", None)
        self.speaker_ids = self.speaker_manager.name_to_id if self.speaker_manager is not None else None
        self.viseme_generator = VisemeGenerator()

    def synthesize(self, text: str, speaker_id: str = "", save_path: str = None):
        """Turn text into a wav file and return byte stream"""
        style_wav = style_wav_uri_to_dict("")
        try:
            wavs = self.synth.tts(text, speaker_name=speaker_id, style_wav=style_wav)
        except Exception as exc:
            print(exc)
        outstream = io.BytesIO()
        self.synth.save_wav(wavs, outstream)
        if save_path:
            self.save_path = save_path
        else:
            self.save_path = os.path.join(
                self.path, f"output/test_speech{speaker_id}.wav")
        self.synth.save_wav(wavs, self.save_path)
        sound = sf.SoundFile(self.save_path)
        speaking_length = sound.frames / sound.samplerate

        visemes = self.viseme_generator.get_visemes(text)
        viseme_length = (speaking_length) / (len(visemes)+1)
        delays = [viseme_length for i in range(len(visemes))]
        return outstream, visemes, delays


def main():
    """Integration testing of Coqui Speaker"""
    example = ("Please call Stella.  Ask her to bring these things with her from the store: "
               "Six spoons of fresh snow peas, five thick slabs of blue cheese, "
               "and maybe a snack for her brother Bob.  We also need a small plastic snake "
               "and a big toy frog for the kids.  She can scoop these things into three red bags, "
               "and we will go meet her Wednesday at the train station.")    

    k = "p267"
    tts = CoquiSpeak()
    _, visemes, delays = tts.synthesize(example, k)

    play_audio_viseme(tts.save_path, visemes, delays)

    # speaker_dict = tts.synth.tts_model.speaker_manager.name_to_id
    # for k, v in speaker_dict.items():
    #     print(k,v)
    # outstream, visemes, delays = tts.synthesize(example, k)

if __name__ == "__main__":
    main()
