import matplotlib.pyplot as plt
import numpy as np

file_ob = open('step.txt', 'r')
list1 = file_ob.readlines()
print(len(list1))
file_ob.close()
reward = []
step = []
for i in range(0, len(list1)):
    if list1[i] == "\n" or list1[i][0] != "S":
        continue
    list1[i] = list1[i].strip('\n')
    print(list1[i].split(":")[2].split(",")[0])
    reward.append(float((list1[i].split(":")[2].split(",")[0])))
    step.append(float((list1[i].split(":")[1].split(",")[0])))
# reward = step
plt.plot(np.linspace(0, len(reward), len(reward)), reward)
plt.xlabel('Episode X')
plt.ylabel('reward')
plt.show()
