import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import re
sns.set(style="white",palette="muted",color_codes=True)
with open("1position_dis-dis.txt") as f:
	a = f.readlines()
reward = []
step = []
learning = True
for i in range(len(a)):
	if a[i][21:26] == 'learn':
		learning = True
	if a[i][0] == 'S' and learning:
		step.append(int(a[i][:9].split(",")[0].split(":")[1]))
		reward_ = float(a[i].split(":")[2].split(",")[0])
		if reward_ > -100:
			reward.append(reward_)

number = []
for i in range(1,21):
	number.append(step.count(i))
	print("number of "+str(i)+" step:", number[i-1])
print("total number:",len(step))
title1 = "grasp success present:{0}%".format(100*(1-number[19]/sum(number)))
print(title1)

fig, axes = plt.subplots(1, 2)
sns.barplot(np.linspace(1,len(number),len(number)),number, ax=axes[0])
axes[0].set_xlabel("Done Step")
axes[0].set_ylabel("Number")
axes[0].set_title(title1)


axes[1].plot(reward)
axes[1].set_xlabel("Episode")
axes[1].set_ylabel("Reward")
axes[1].set_title("the episode after begining learning")
plt.show()
