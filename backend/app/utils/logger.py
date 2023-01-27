from datetime import datetime
from time import sleep
import os

class Logger():
    def __init__(self, folder="./logs") -> None:
        self.folder = folder
        if not os.path.exists(folder):
             os.makedirs(folder)
             print(f"Making folder {folder}")
        now = datetime.now()
        print("now =", now)
        dt_string = now.strftime("%d_%m_%Y__%H_%M_%S")
        print("date and time =", dt_string)
        self.f = open(f"{folder}/{dt_string}.txt", "x")

    def log(self, string, printnow=True):
        if printnow: print(string)
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y__%H_%M_%S")
        self.f.write(f"{dt_string}: {string}\n")

    def log_sound(self, file_clip):
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y__%H_%M_%S")
        file_clip.export(f"{self.folder}/{dt_string}.wav", format="wav")




if __name__ == "__main__":
    l = Logger()
    i=0
    while True:
        i+=1
        l.log(f"test {i}")
        sleep(1)