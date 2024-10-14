'''
Usage: semseg2png input.npz output.png
This will convert semseg npz in our dataset to a visible png file.
'''

import os
import torch
import numpy as np
import json
import cv2
from tqdm import *
import sys

args = list(sys.argv)

# map from semseg label to color
trans = [
	np.array([0, 0, 0]),
	np.array([70, 70, 70]),
	np.array([100, 40, 40]),
	np.array([55, 90, 80]),
	np.array([220, 20, 60]),
	np.array([153, 153, 153]),
	np.array([157, 234, 50]),
	np.array([128, 64, 128]),
	np.array([244, 35, 232]),
	np.array([107, 142, 35]),
	np.array([0, 0, 142]),
	np.array([102, 102, 156]),
	np.array([220, 220, 0]),
	np.array([70, 130, 180]),
	np.array([81, 0, 81]),
	np.array([150, 100, 100]),
	np.array([230, 150, 140]),
	np.array([180, 165, 180]),
	np.array([250, 170, 30]),
	np.array([110, 190, 160]),
	np.array([170, 120, 50]),
	np.array([45, 60, 150]),
	np.array([145, 170, 100]),
	np.array([0, 0, 0]),
	np.array([0, 0, 0]),
	np.array([0, 0, 0]),
	np.array([0, 0, 0]),
	np.array([0, 0, 0]),
	np.array([0, 0, 0]),
	np.array([0, 0, 0]),
]


tp = np.load(args[1])

arr_ori = tp['arr_0']

print(arr_ori.shape)

al = []

for i in tqdm(range(1080)):
	a = []
	for j in range(1920):
		a.append(np.flip(trans[arr_ori[i][j]]))

	al.append(a)

al = np.array(al)

cv2.imwrite(os.path.join(os.getcwd(), args[2]), al)