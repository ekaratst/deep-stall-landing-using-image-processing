import time
'''
def show_time():
	named_tuple = time.localtime() # get struct_time
	time_string = time.strftime("%S", named_tuple)
	print(time_string)

def time2():
	result = time.localtime(1545925769)
	print("result:", result)
	print("\nyear:", result.tm_year)
	print("tm_hour:", result.tm_hour)
	
show_time()
now = time.ctime(time.time())
#result1 = time.time()
#print(result1)
print(now)

count = 5
while count:
	print(count)
	time.sleep(1)
	count = count - 1
time = time.ctime(time.time())
print(time)
'''

import time

start = time.time()
print(start)
count = 0
while True:
	now = time.time()
	print(count)
	if now - start >= 5:
		break
	count = count +1
		
print("stop")




