#!/usr/bin/python3

import _thread
import time

# Define a function for the thread
def t1():
   print("1")
   while True:
      print("while t1")

def t2():
   print("2")
   while True:
      print("while t2")


# Create two threads as follows
try:
   _thread.start_new_thread( t1, ( ) )
   _thread.start_new_thread( t2, ( ) )
except:
   print ("Error: unable to start thread")

while 1:
   pass
