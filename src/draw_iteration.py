#!/usr/bin/python
# encoding: utf-8

# from PIL import Image
# from skimage import io,data,color
# import os.path
import matplotlib.pyplot as plt
# import matplotlib.image as img
import math as m
import numpy as np


def read_data_ref(data_file, points):
    file = open(data_file, "r")
    try:
        while (True):
            line = file.readline()
            if not line:
                break
            point = [ float(i) for i in line.split()]
            points.append(point)
    except:
        raise
    finally:
        file.close()

def read_data_iter(data_file, data):
    file = open(data_file, "r")
    try:
        while (True):
            line = file.readline()
            if not line:
                break
            d = [ float(i) for i in line.split()]
            data.append(d)
    except:
        raise
    finally:
        file.close()


if __name__ =='__main__':
    point_num = 937
    # nframes = input("请输入帧数：")
    # niter = input("请输入迭代次数：")
    # ref = input("请输入参考帧号：")
    # sen = input("请输当前帧号：")

    # nframes = 484
    niter = 27
    ref = 419
    sen = 420

    poses_ref = []
    poses_sen = []
    poses_iter = [[] for i in range(niter)]
    gp_ref_x = []
    gp_ref_y = []
    gp_sen_x = []
    gp_sen_y = []
    gp_iter_x = [[] for i in range(niter)]
    gp_iter_y = [[] for i in range(niter)]

    data_ref_file = "/home/vance/slam_ws/pl-icp/bin/0824-6/frame-" + str(ref) + "-ref.txt"
    read_data_ref(data_ref_file, poses_ref)
    for pose in poses_ref:
        if m.isnan(pose[0]) or m.isnan(pose[1]):
            continue
        gp_ref_x.append(pose[0])
        gp_ref_y.append(pose[1])

    data_sen_file = "/home/vance/slam_ws/pl-icp/bin/0824-6/frame-" + str(sen) + "-sen.txt"
    read_data_ref(data_sen_file, poses_sen)
    for pose in poses_sen:
        if m.isnan(pose[0]) or m.isnan(pose[1]):
            continue
        gp_sen_x.append(pose[0])
        gp_sen_y.append(pose[1])

    data_iter_file = []
    for i in range(niter):
        fn = "/home/vance/slam_ws/pl-icp/bin/0824-6/frame-" \
             + str(sen) + "-to-" + str(ref) + "-iter-" + str(i) + ".txt"
        data_iter_file.append(fn)
        read_data_iter(data_iter_file[i], poses_iter[i])
        for pose in poses_iter[i]:
            if m.isnan(pose[0]) or m.isnan(pose[1]):
                continue
            gp_iter_x[i].append(pose[0])
            gp_iter_y[i].append(pose[1])

    # for i in range(point_num):



    plt.plot(gp_ref_x, gp_ref_y, '.r', gp_sen_x, gp_sen_y, '.g')
    plt.title("first poses for two frames")
    plt.show()
    plt.plot(gp_sen_x, gp_sen_y, '.r', gp_iter_x[0], gp_iter_y[0], '.g')
    plt.title("first poses for two frames")
    plt.show()
    for i in range(niter):
        plt.plot(gp_ref_x, gp_ref_y, '.r', gp_iter_x[i], gp_iter_y[i], '.g')
        title = "poses for No." + str(i) + " iteration"
        plt.title(title)
        plt.show()

    # plt.xlim(-6, 6)
    # plt.ylim(-2, 2)

