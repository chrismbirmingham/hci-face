from logger import Logger
import time


l = Logger( folder="./logs" )

@l.log_function
def test_logger(a,b, f="test"):
    time.sleep(1)
    c = unlogged_function(a,b)
    time.sleep(1)
    
    print(c)
    time.sleep(15)
    print("all done!")
    return c
    
    
    
def unlogged_function(a,b):
    print("I am not logged")
    return a+b
    
l.log("start of test")
test_logger(1,2, f="test2")
l.log("end of test")