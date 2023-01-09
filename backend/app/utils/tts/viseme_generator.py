from phonemizer.backend import EspeakBackend
import pandas as pd
from pathlib import Path
import os


class VisemeGenerator:
    def __init__(self, convertion_table) -> None:
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
                    print(f"Viseme for: {ipa} NOT FOUND")
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
        print(o)
        return o


    def get_visemes(self, text, return_phonemes = False):
        if type(text) == str:
            text = [text]
        print(f"String to process: {text}")
        phonemized = self.backend.phonemize(text, strip=False)[0]
        print(f"Phonemes: {phonemized}")
        s = self.process_phoneme_string(phonemized) + [' ']
        v = []
        for p in s:
            vis = self.get_viseme(p)
            if vis == "IDLE":
                v.append(vis)
            v.append(vis)

        if return_phonemes:
            return v, s
        return v


if __name__ == "__main__":
    text = ["Hello, world! Welcome to the arena?"]
    vg = VisemeGenerator("./phoneme-viseme_map.csv")
    v,s = vg.get_visemes(text, True)

    print(len(v), len(s))
    for i in range(len(v)):
        print(i, s[i], v[i])