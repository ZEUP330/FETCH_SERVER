#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np

# 超参数
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
ACTION_SERVER = 'gripper_controller/gripper_action'
# device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class Robot(object):
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons
    dt = 0.005  # 转动的速度和 dt 有关
    action_bound = [-1, 1]  # 转动的角度范围
    state_dim = 3  # 3个观测值
    action_dim = 3  # 3个动作

    def __init__(self):
        self.Box_position = [0.5908450974804389, 0.0754887624414724, 0.75]  # [0.6, 0.1, 0.65]
        self.end_goal = [0.0, 0.0, 0.0]
        # 初始化reward
        self.reward = 0
        self.mini_dis = 1000

    def get_state(self):
        return self.end_goal

    def random_box_position(self):
        tmp = np.random.uniform(low=-0.27, high=0.27, size=3)
        self.Box_position = tmp.tolist()
        self.Box_position[0] += 0.52
        self.Box_position[1] *= 1.2
        self.Box_position[2] = 0.75
        return self.Box_position

    def read_box_position(self):
        return self.Box_position

    def test_step(self, action, var):
        # self.cont+=1
        done = False
        success = False
        # act = action[0]
        self.end_goal += action[0]
        # self.end_goal[0] = np.clip(np.random.normal(self.end_goal[0], var), -1, 1)
        # self.end_goal[1] = np.clip(np.random.normal(self.end_goal[1], var), -1, 1)
        # self.end_goal[2] = np.clip(np.random.normal(self.end_goal[2], var), -1, 1)
        self.end_goal[0] = np.random.normal(self.end_goal[0], var) % 0.4 + 0.52
        self.end_goal[1] = np.random.normal(self.end_goal[0], var) % 0.7 - 0.35
        self.end_goal[2] = 0.75
        # self.end_goal = [0.5908450974804389, 0.0754887624414724, 0.75]
        x = self.end_goal[0]
        y = self.end_goal[1]
        z = self.end_goal[2]
        # print(x, y)
        dis = math.sqrt(math.pow(x - self.Box_position[0], 2)
                        + math.pow(y - self.Box_position[1], 2))
        reward = -dis
        if dis < 0.03:  # 阈值，可调
            done = True
            success = True
        self.dis = dis
        if dis < self.mini_dis:
            self.mini_dis = dis
        # if(self.cont>5000):
        #     print(x,y, self.Box_position, dis, reward)
        new_position = [x, y, z]
        return new_position, reward, done, success
