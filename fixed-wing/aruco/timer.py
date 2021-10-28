import time, threading, sys

def hello():
    print("exit")
    sys.exit()

t = threading.Timer(3, hello)
t.start()
# while True:
#     print("run")