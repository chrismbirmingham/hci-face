"""Audio viseme player

"""

import subprocess
import os
import sys
import time


def play_audio_viseme(save_path, visemes, delays):
    """Plays audio and visemes at the same time

    Args:
        save_path (path): path to audio file
        visemes (list): list of visemes
        delays (list): list of delays
    """
    # Play the audio using the platform's default player
    if sys.platform == "win32":
        os.startfile(save_path)
    else:
        # The following works on macOS and Linux. (Darwin = mac, xdg-open = linux).
        opener = "open" if sys.platform == "darwin" else "xdg-open"

        subprocess.call([opener, save_path])
        for ind, sleep in enumerate(delays):
            time.sleep(sleep)
            print(sleep, visemes[ind])
