#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np
import sys


def get_value(strline, name):
    if strline.find(name) < 0:
        raise Exception("File format not matching the parser expectation", name)

    return strline.replace(name, "", 1)


def get_pose(line):
    ss = line.split()
    return np.array([float(ss[0]), float(ss[1]), float(ss[2])])


class MPrim:
    def __init__(self, f):
        self.primID = int(get_value(f.readline(), "primID:"))
        self.startAngle = int(get_value(f.readline(), "startangle_c:"))
        self.endPose = get_pose(get_value(f.readline(), "endpose_c:"))
        self.cost = float(get_value(f.readline(), "additionalactioncostmult:"))
        self.nrPoses = int(get_value(f.readline(), "intermediateposes:"))
        poses = []
        for _ in range(self.nrPoses):
            poses.append(f.readline())
        self.poses = np.loadtxt(poses, delimiter=" ")
        self.cmap = plt.get_cmap("nipy_spectral")

    def plot(self, nr_angles):
        plt.plot(self.poses[:, 0], self.poses[:, 1], c=self.cmap(float(self.startAngle) / nr_angles))


class MPrims:
    def __init__(self, filename):
        f = open(filename, "r")

        self.resolution = float(get_value(f.readline(), "resolution_m:"))
        self.nrAngles = int(get_value(f.readline(), "numberofangles:"))
        self.nrPrims = int(get_value(f.readline(), "totalnumberofprimitives:"))

        self.prims = []
        for _ in range(self.nrPrims):
            self.prims.append(MPrim(f))

        f.close()

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xticks(np.arange(-1, 1, self.resolution))
        ax.set_yticks(np.arange(-1, 1, self.resolution))
        for prim in self.prims:
            prim.plot(self.nrAngles)
        plt.grid()
        plt.show()


prims = MPrims(sys.argv[1])
prims.plot()
