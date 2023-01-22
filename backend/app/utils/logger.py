from datetime import datetime
from time import sleep


class Logger():
    def __init__(self, folder="./logs") -> None:
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



if __name__ == "__main__":
    l = Logger()
    i=0
    while True:
        i+=1
        l.log(f"test {i}")
        sleep(1)