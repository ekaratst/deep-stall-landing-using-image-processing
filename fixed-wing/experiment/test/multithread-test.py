#!/usr/bin/python3

import _thread
import time

a=20
# Define a function for the thread
def print_time( threadName, delay):
   count = 0
   while count < 5:
      time.sleep(delay)
      count += 1
      print ("%s: %s" % ( threadName, time.ctime(time.time()) ))
      print(a)

def print10(threadName):
    while True:
        print ("%s: %s" % ( threadName, time.ctime(time.time()) ))
        time.sleep(1)
        return 10

# Create two threads as follows
try:
   y = print10("Thread-3")
   _thread.start_new_thread( print_time, ("Thread-1", 2, ) )
   _thread.start_new_thread( print_time, ("Thread-2", 4, ) )
  
   print(y)
except:
   print ("Error: unable to start thread")

while 1:
   pass