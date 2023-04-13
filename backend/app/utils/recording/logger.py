#!/usr/bin/env python3
"""Basic logger used in place of print statements

Also supports logging sound clips to be saved 
with their timestamp
"""

import asyncio
from datetime import datetime
import os
import sys
import time
from functools import wraps

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
        dt_string = self.get_date_str()
        full_string = f"{dt_string}: {string}\n"
        if printnow:
            print(full_string, end="")
        self.open_file.write(full_string)

    def log_sound(self, file_clip):
        """Saves sound file to folder with timestamp name"""
        dt_string = self.get_date_str()
        file_clip.export(f"{self.folder}/{dt_string}.wav", format="wav")


    def log_function(self, func):
        """A decorator function that logs the input and runtime of a function

        Args:
            func (function): function to log
        """
        @wraps(func)
        async def wrapper(*args, **kwargs):
            old_stdout = sys.stdout
            start = time.time()
            self.log(f"''{func.__name__}'' called with args: {args} and kwargs: {kwargs}")
            sys.stdout = CustomOut(old_stdout, self.open_file)
            if asyncio.iscoroutinefunction(func):
                result = await func(*args, **kwargs)
            else:
                result = func(*args, **kwargs)
            sys.stdout = old_stdout
            total = round(time.time()-start,3)
            self.log(f"''{func.__name__}'' returned: {result} after {total} seconds")
            return result
        return wrapper


class CustomOut():
    """A custom stdout that prints and logs to a file"""
    def __init__(self, old_stdout, open_file):
        # self.buffer = io.StringIO()
        self.old_stdout = old_stdout
        self.open_file = open_file
        self.counter = 0

    def write(self, string):
        if string == "\n":
            self.open_file.write(string)
            self.old_stdout.write(string)
            return
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y__%H_%M_%S")
        full_string = f"{dt_string}: {string}"
        self.open_file.write(full_string)
        self.open_file.flush()
        self.old_stdout.write(full_string)
        self.counter += 1

