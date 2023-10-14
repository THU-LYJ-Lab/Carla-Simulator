import numpy as np
from tqdm import *

for i in tqdm(range(6)):
	for j in range(1000):
		a = np.load('masks/camera_%d/%.8d.npz' % (i, j))
		np.savez_compressed('masks/camera_%d/%.8d.npz' % (i, j), arr_0 = a['array_0'])