import matplotlib.pyplot as plt
import numpy as np
import re
with open("data.txt") as f:
	a = f.readlines()
data = []
#print(a)
for i in range(len(a)):
	if a[i][0] == 'S':#  and i % 10 == 0 :
		data.append(int(a[i][:9].split(",")[0].split(":")[1]))
		#aa = float(a[i].split(":")[3].split(",")[0])
		#if aa > -100:
		#	data.append(aa)
#print(np.linspace(1,len(data),len(data))) 
number = []
for i in range(1,21):
	number.append(data.count(i))
	print("number of "+str(i)+" step:", number[i-1])
print("total number:",len(data))
print("Bigger than 20:",len(data)-sum(number))
#plt.plot(data)
#plt.show()
