#!/usr/bin/env python3
"""Basic logger used in place of print statements

Also supports logging sound clips to be saved 
with their timestamp
"""

from datetime import datetime
import os

class Logger():
    """Logs print statements to a file and sound clips to a folder

    File logging is dynamic so that it will be saved even if there is
    a crash.
    """
    def __init__(self, folder="./logs") -> None:
        self.folder = folder
        if not os.path.exists(folder):
            os.makedirs(folder)
            print(f"Making folder {folder}")
        dt_string = self.get_date_str()
        print("date and time =", dt_string)
        self.open_file = open(f"{folder}/{dt_string}.txt", "x")

    def get_date_str(self) -> str:
        """Returns current date as formatted str

        Returns:
            str: day_month_year__hour_minute_second
        """
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y__%H_%M_%S")
        return dt_string

    def log(self, string, printnow=True):
        """Logs print statements to file with timestamp header"""
        if printnow:
            print(string)
        dt_string = self.get_date_str()
        self.open_file.write(f"{dt_string}: {string}\n")

    def log_sound(self, file_clip):
        """Saves sound file to folder with timestamp name"""
        dt_string = self.get_date_str()
        file_clip.export(f"{self.folder}/{dt_string}.wav", format="wav")
