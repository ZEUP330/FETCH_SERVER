# -*- coding: utf-8 -*-
from __future__ import division
import torch
import torch.nn as nn
from torch.autograd import Variable
import torch.utils.data as Data
from RGBD_XYZ.unet_model import UNet
import os
import cv2
import matplotlib.pyplot as plt
from torch.utils.data import TensorDataset, Dataset, DataLoader
import numpy as np
EPOCH = 100
STEP = 1000
BATCH_SIZE = 15
LR = 0.0005


def test_read():
    position, x1, y1, z1 = read_data(size=4)
    for i in range(len(position)):
        print(position)
        rgb, dep = read_img_dep(position[i][:-4])
        plt.imshow(rgb)
        plt.show()
        plt.imshow(dep)
        plt.show()
        x1 = float(position[i][1:-11].split(" ")[0][:-1])
        y1 = float(position[i][1:-11].split(" ")[1])
        print(x1, y1)


def random_read_():
    tar, pos = data_prepare()
    index = np.random.randint(1, 9999)
    while int(pos[index].split(".")[2][:2]) > 27:
        index += 1
        index %= 9999
    while True:
        index += 1
        index %= 9999
        rgb, dep = read_img_dep(pos[index][:-4])
#         print(pos[index][:-4])
        plt.imshow(rgb)
        plt.show()
        xy = np.mean(np.argwhere(rgb[:, :, :] == 0), axis=0).astype(np.float32)
        ground_true = np.zeros((1, 224, 224))
        x = np.argwhere(rgb[:, :, 2] == 0)[:, 0]
        y = np.argwhere(rgb[:, :, 2] == 0)[:, 1]
        ground_true[:, x, y] = 1
        if np.isnan(np.max(xy)):
            continue
        break
    img = torch.from_numpy(np.expand_dims(rgb, axis=0).astype(np.float32)).permute(0, 3, 1, 2).cuda()

    ground_true = torch.from_numpy(ground_true.astype(np.float32)).cuda().long()
    return img, ground_true, xy[:2]


def read_img_dep(file_name):
    dep = np.load("D:/images/images/dep/" + file_name + ".npy").reshape((224, 224, 1))
    dep[np.isnan(dep)] = 0
    dep = np.concatenate((dep, dep, dep), axis=2)
    return cv2.imread("D:/images/images/rgb/" + file_name + ".png"), dep


def read_data(file_dir="D:/images/images/dep/", size=1):
    position = []
    number = 0
    x1 = []
    y1 = []
    z1 = []
    for root_name, not_use, files_name in os.walk(file_dir):
        # print "load data from:{0},there are {1} picture".format(root_name, len(files_name))
        for i in files_name:
            position.append(i)
            x1.append(float(i[1:-11].split(" ")[0][:-1]))
            y1.append(float(i[1:-11].split(" ")[1]))
            z1.append(float(0.75))
            number += 1
            if number == size:
                break
    return position, x1, y1, z1


def data_prepare():
    position, x, y, z = read_data(size=10000)
    x = np.array(x).reshape(-1, 1)
    y = np.array(y).reshape(-1, 1)
    z = np.array(z).reshape(-1, 1)
    return np.concatenate((x, y, z), axis=1), position


class MyDataset(Dataset):
    def __init__(self):
        self.tar, self.pos = data_prepare()
        self.y_data = torch.from_numpy(self.tar)
        self.len = self.tar.shape[0]

    def __getitem__(self, index):
        self.index = np.random.randint(1, 9999)
        while int(self.pos[self.index].split(".")[2][:2]) > 27:
            self.index += 1
            self.index %= 9999
        while True:
            self.index += 1
            self.index %= 9999
            self.rgb, self.dep = read_img_dep(self.pos[self.index][:-4])
            self.xy = np.mean(np.argwhere(self.rgb[:, :, :] == 0), axis=0).astype(np.float32)
            self.ground_true = np.zeros((1, 224, 224))
            self.x = np.argwhere(self.rgb[:, :, 2] == 0)[:, 0]
            self.y = np.argwhere(self.rgb[:, :, 2] == 0)[:, 1]
            self.ground_true[:, self.x, self.y] = 1
            if np.isnan(np.max(self.xy)):
                continue
            break
        ground_true = torch.from_numpy(self.ground_true.astype(np.float32)).cuda()
        img = torch.from_numpy(np.expand_dims(self.rgb, axis=0).astype(np.float32)).permute(0, 3, 1, 2).cuda()
        return img.squeeze(dim=0), ground_true.squeeze(dim=0).long()

    def __len__(self):
        return self.len


class rgb_xy():
    def __init__(self):
#         print("-------{Building Network}-------")
        self.unet = UNet(n_channels=3, n_classes=2).cuda()  # print unet
#         print("-------{Build finish!}-------")
        self.optimizer = torch.optim.Adam(self.unet.parameters(), lr=LR)
        # self.optimizer = torch.optim.SGD(self.unet.parameters(), lr=LR)
        # self.loss_func = nn.L1Loss()
        self.loss_func = nn.CrossEntropyLoss()

        self.data = MyDataset()
        self.train_loader = torch.utils.data.DataLoader(dataset=self.data, batch_size=BATCH_SIZE, shuffle=True)
        self.load_model(model_path="D:/GitDesktop/FETCH_SERVER/RGBD_XYZ/rgb2x.pkl")

    def train(self):
#         print("-------{Start Trian}-------")
        for epoch in range(EPOCH):
            for batch_idx, (rgb, ground_true) in enumerate(self.train_loader):
                # print batch_idx
                b_rgb = Variable(rgb).cuda()  # todo
                b_ground_true = Variable(ground_true).cuda()
                output = self.unet(b_rgb).float()
                loss = self.loss_func(output, b_ground_true)
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                if batch_idx % 10 == 0:
                    test_img, test_ground_true, xy = random_read_()
                    test_out = self.unet(test_img)
                    accuracy = float(self.loss_func(test_out, test_ground_true))
                    print("Epoch:", epoch, "|train loss: %.4f" % loss.cpu().data.numpy(), \
                        "|test accuracy:%.4f" % (1 - accuracy))
                    black = np.zeros((224, 224, 1))
                    numpy_out = test_out[0].permute(1, 2, 0).cpu().data.numpy()
                    numpy_out[numpy_out > 0.5] = 100
                    numpy_out[numpy_out < 0.5] = 0
                    out = np.concatenate((numpy_out.astype(np.int64), black), axis=2)
                    plt.clf()
                    # plt.imshow(out)
                    # plt.show()
            torch.save(self.unet, "rgb2x.pkl")

    def load_model(self, model_path="rgb2x.pkl"):
        self.unet = torch.load(model_path)

    def detect(self, rgb):
        # img = torch.from_numpy(rgb.astype(np.float32)).cuda()
        img = torch.from_numpy(np.expand_dims(rgb, axis=0).astype(np.float32)).permute(0, 3, 1, 2).cuda()
        test_out = self.unet(img)
        black = np.zeros((224, 224, 1))
        numpy_out = test_out[0].permute(1, 2, 0).cpu().data.numpy()
        numpy_out[numpy_out > 0.5] = 100
        numpy_out[numpy_out < 0.5] = 0
        out = np.concatenate((numpy_out.astype(np.int64), black), axis=2)
        # plt.clf()
        # plt.imshow(out)
        # plt.show()
        xy = np.mean(np.argwhere(out[:, :, 0] == 0), axis=0).astype(np.float32)
        return xy


if __name__ == "__main__":
    method = rgb_xy()
    # method.train()
    test_img, dep = read_img_dep("[0.5513610201719624, 0.1887571504509108, 0.75]")
    print(method.detect(test_img))
