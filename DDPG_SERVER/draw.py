import matplotlib.pyplot as plt
import numpy as np

file_ob = open('step.txt', 'r')
list1 = file_ob.readlines()
file_ob.close()
for i in range(0, len(list1)):
    list1[i] = list1[i].rstrip('\n')

plt.hist(list1, bins=40, density=0, facecolor="blue", edgecolor="black", alpha=0.7)
plt.xlabel('Episode X')
plt.ylabel('Steps')
plt.show()
