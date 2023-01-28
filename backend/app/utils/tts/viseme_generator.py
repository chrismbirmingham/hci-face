from phonemizer.backend import EspeakBackend
import pandas as pd
from pathlib import Path
import os


class VisemeGenerator:
    def __init__(self, convertion_table, log=False) -> None:
        self.log = log
        self.path = Path(__file__).parent  
        conversion_table_path = os.path.join(self.path, convertion_table)

        self.backend = EspeakBackend('en-us', preserve_punctuation=True, with_stress=False)
        
        self.joint_df = pd.read_csv(conversion_table_path, header=0)
        self.joint_df.set_index("IPA")
        pass

    def get_viseme(self, ipa):
        try:
            d = self.joint_df[self.joint_df['IPA']==ipa]
            if not len(d):
                d = self.joint_df[self.joint_df['Alternative IPA'] == ipa]
                if not len(d):
                    if self.log: print(f"Viseme for: {ipa} NOT FOUND")
            viseme = d["SimpleViseme"].values[0]
        except:
            viseme = "IDLE"
        return viseme

    def process_phoneme_string(self, s):
        o = []
        for l in s:
            if l != "ː":
                o.append(l)
            else:
                o[-1] = o[-1] + "ː" # note this is a special character, not a colon
        if self.log: print(o)
        return o


    def get_visemes(self, text, return_phonemes = False):
        if type(text) == str:
            text = [text]
        if self.log: print(f"String to process: {text}")
        phonemized = self.backend.phonemize(text, strip=False)[0]
        if self.log: print(f"Phonemes: {phonemized}")
        phonemes = self.process_phoneme_string(phonemized) + [' ']
        visemes = []
        for p in phonemes:
            vis = self.get_viseme(p)
            if vis == "IDLE":
                visemes.append(vis)
            visemes.append(vis)

        if return_phonemes:
            return visemes, phonemes
        return visemes


if __name__ == "__main__":
    text = ["Hello, world! Welcome to the arena?"]
    vg = VisemeGenerator("./.phoneme-viseme_map.csv")
    visemes,phonemes = vg.get_visemes(text, True)

    print(len(visemes), len(phonemes))
    m = min(len(visemes),len(phonemes))
    for i in range(m):
        print(i, phonemes[i], visemes[i])