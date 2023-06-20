import os
import numpy as np
import copy
from sklearn.mixture import GaussianMixture

from utils.save_gmm import save

def parse_mine_dataset():
    cwd = os.getcwd()
    SANDBOX_NAME = 'gira3d-registration'
    matches = cwd.split(SANDBOX_NAME)
    GIRA3D_REGISTRATION_SANDBOX = matches[0] + SANDBOX_NAME
    PCLD_DIR = GIRA3D_REGISTRATION_SANDBOX + '/data/mine_001_part3/pointclouds/'
    FIRST_SCAN = 240
    LAST_SCAN = 660
    N_COMPONENTS = 70

    count = 1
    for i in range(FIRST_SCAN, LAST_SCAN):
        print('Working on scan ' + str(i) + ' out of ' + str(LAST_SCAN))
        pointcloud_idx = i
        save_idx = count
        file_to_load = PCLD_DIR + str(pointcloud_idx) + '.txt'
        file_to_save = './gmms/' + f"{save_idx:06d}" + '.gmm'
        g = GaussianMixture(N_COMPONENTS)
        data = np.loadtxt(file_to_load, delimiter=',')
        g.fit(data)
        save(file_to_save, g)
        count = count + 1

if __name__ == "__main__":
    parse_mine_dataset()
