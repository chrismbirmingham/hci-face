""" Generate Visemes from text.

This module converts text to phonemes and then phonemes to visemes.

"""

from pathlib import Path
import os
from phonemizer.backend import EspeakBackend
import pandas as pd


class VisemeGenerator:
    """Contains the functionality to convert text to visemes
    
    Or to convert between viseme types
    """

    def __init__(self, convertion_table="./resources/.phoneme-viseme_map.csv", log=False) -> None:
        
        self.log = log
        self.path = Path(__file__).parent
        conversion_table_path = os.path.join(self.path, convertion_table)

        self.backend = EspeakBackend('en-us', preserve_punctuation=True, with_stress=False)

        self.joint_df = pd.read_csv(conversion_table_path, header=0)
        self.joint_df.set_index("IPA")

    def get_viseme(self, ipa, type='IPA'):
        """Converts individual phoneme to a viseme"""
        try:
            viseme_dict = self.joint_df[self.joint_df[type]==ipa]
            if len(viseme_dict) < 1:
                viseme_dict = self.joint_df[self.joint_df['Alternative IPA'] == ipa]
                if len(viseme_dict) < 1:
                    if self.log:
                        print(f"Viseme for: {ipa} NOT FOUND")
            if ipa == "sil":
                viseme = "IDLE"
            else:
                viseme = viseme_dict["SimpleViseme"].values[0]
        except Exception as exc:
            print(f"Get viseme [{ipa}] failed: {exc}")
            viseme = "IDLE"
        return viseme

    def process_phoneme_string(self, phonemes_string):
        """Converts phoneme string to list

        Handles a bug with a special character as well
        """
        phoneme_list = []
        for phoneme in phonemes_string:
            if phoneme != "ː":
                phoneme_list.append(phoneme)
            else:
                phoneme_list[-1] = phoneme_list[-1] + "ː"
                                    # note this is a special character, not a colon
        if self.log:
            print(phoneme_list)
        return phoneme_list

    def convert_aws_visemes(self, visemes):
        new_visemes = []
        for vis in visemes:
            new_visemes.append(self.get_viseme(vis, type="Viseme"))
        return new_visemes

    def get_visemes(self, sentence, return_phonemes = False):
        """Process a sentence or list of sentences into visemes"""
        if isinstance(sentence, str):
            sentence = [sentence]
        if self.log:
            print(f"String to process: {sentence}")
        phonemized = self.backend.phonemize(sentence, strip=False)[0]
        if self.log:
            print(f"Phonemes: {phonemized}")
        phoneme_list = self.process_phoneme_string(phonemized) + [' ']
        visemes_list = []
        for phoneme in phoneme_list:
            vis = self.get_viseme(phoneme)
            if vis == "IDLE":# Append Idles twice to give better pauses
                visemes_list.append(vis)
            visemes_list.append(vis)

        if return_phonemes:
            return visemes_list, phoneme_list
        return visemes_list

def main():
    """Integration testing for viseme generator"""
    text = ["Hello, world! Welcome to the arena?"]
    gen = VisemeGenerator()
    visemes,phonemes = gen.get_visemes(text, True)

    print(len(visemes), len(phonemes))
    min_phone_vis = min(len(visemes),len(phonemes))
    for i in range(min_phone_vis):
        print(i, phonemes[i], visemes[i])

if __name__ == "__main__":
    main()
