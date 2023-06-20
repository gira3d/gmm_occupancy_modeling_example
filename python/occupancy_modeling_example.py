#!/usr/bin/env python
import copy
import numpy as np
import numpy.matlib
import os
from os.path import expanduser
import pickle
from tqdm import tqdm
from sklearn.mixture import GaussianMixture
import yaml

from utils.open3d_visualizer import Open3DVisualizer
from utils.QuatToR import QuatToR
from utils.pose_compose import pose_compose
from grid3d import Grid3D, Parameters, Point


def convert_dict_to_params(d):
    p = Parameters()
    p.width = d['map']['width']
    p.height = d['map']['height']
    p.depth = d['map']['depth']
    p.resolution = d['map']['resolution']
    p.origin.x = d['map']['origin_x']
    p.origin.y = d['map']['origin_y']
    p.origin.z = d['map']['origin_z']
    p.min_clamp = d['map']['clamping_min']
    p.max_clamp = d['map']['clamping_max']
    p.free_threshold = d['map']['free_threshold']
    p.occupancy_threshold = d['map']['occupancy_threshold']
    p.prob_hit = d['map']['probability_hit']
    p.prob_miss = d['map']['probability_miss']
    p.block_size = d['map']['block_size']
    p.lock_at_max_clamp = d['map']['lock_at_max_clamp']
    p.track_changes = d['map']['track_changes']

    return p


def read(filepath):
    data = np.loadtxt(filepath, delimiter=',')
    means = data[:, 0:3]
    c = data[:, 3:12]
    weights = data[:, 12]

    covs = np.zeros((np.shape(c)[0], 3, 3))
    for i in range(0, np.shape(c)[0]):
        covs[i, :, :] = np.reshape(c[i, :], (3, 3))
    return means, covs, weights


def load_gmm_from_file(filepath):
    means, covs, weights = read(filepath)
    gmm = GaussianMixture(n_components=len(weights))
    gmm.means_ = means
    gmm.covariances_ = covs
    gmm.weights_ = weights / weights.sum()
    return gmm


def main():
    cwd = os.getcwd()
    SANDBOX_NAME = 'gira3d-occupancy-modeling'
    matches = cwd.split(SANDBOX_NAME)
    SANDBOX = matches[0] + SANDBOX_NAME
    DATASET = 'mine_001_part3'
    DATA_DIR = SANDBOX + '/data/' + DATASET + '/'
    RESULTS_DIR = DATA_DIR + 'results/'
    GMM_DIR = DATA_DIR + '100_components/'
    MAX_RANGE = 15.0

    FIRST_SCAN = 1
    LAST_SCAN = 1055

    p = Parameters()
    with open('../config/grid3d.yaml') as file:
        params = yaml.safe_load(file)
        p = convert_dict_to_params(params)
    TRIMMED_MAX_RANGE = MAX_RANGE - p.resolution
    OCCUPANCY_THRESH = p.occupancy_threshold

    grid = Grid3D(p)

    K, Tbc, odometry, pointclouds = pickle.load(
        open(DATA_DIR + 'odometry.pkl', 'rb'))

    print('Loading data.')
    all_pts = []
    for i in tqdm(range(FIRST_SCAN, LAST_SCAN, 5)):

        gmmfile = GMM_DIR + str(i) + '.gmm'
        gmm = load_gmm_from_file(gmmfile)
        ret = gmm.sample(2e4)
        pts = ret[0]

        _, idx = odometry.closest_time(pointclouds.times[i])
        Twb = np.eye(4)
        Twb[0:3, 0:3] = QuatToR(odometry.orientations[:, idx])
        Twb[0:3, 3] = odometry.positions[:, idx]
        Twc = pose_compose(Twb, Tbc)

        R = Twc[0:3, 0:3]
        t = np.reshape(Twc[0:3, 3], (3, 1))
        pts = np.matmul(R, np.transpose(pts)) + \
            np.matlib.repmat(t, 1, np.shape(pts)[0])
        pts = pts.transpose()

        for j in range(0, np.shape(pts)[0]):
            st = Point(t[0, 0], t[1, 0], t[2, 0])
            en = Point(pts[j, 0], pts[j, 1], pts[j, 2])
            grid.add_ray(st, en, TRIMMED_MAX_RANGE)

        if i == FIRST_SCAN:
            all_pts = pts
        else:
            all_pts = np.vstack((all_pts, pts))

    print('Reconstructing occupied areas.')
    n = p.width*p.height*p.depth
    all_pts = None
    for i in tqdm(range(0, p.width*p.height*p.depth)):
        if grid.get(i).logodds > OCCUPANCY_THRESH:
            pt = grid.get_point(i)
            if all_pts is None:
                all_pts = np.array([pt.x, pt.y, pt.z])
            else:
                all_pts = np.vstack((all_pts, np.array([pt.x, pt.y, pt.z])))

    print('Done')
    viz = Open3DVisualizer()
    viz.plot3d(all_pts)


if __name__ == '__main__':
    main()
