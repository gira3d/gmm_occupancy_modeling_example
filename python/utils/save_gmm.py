#!/usr/bin/env python
import numpy as np

def save(filepath, gmm):

    n_components = gmm.n_components
    data = np.zeros([n_components, 13])
    for i in range(0, n_components):
        data[i, 0:3] = gmm.means_[i,:]
        data[i, 3:12] = gmm.covariances_[i].flatten()
        data[i, 12] = gmm.weights_[i]
    np.savetxt(filepath, X=data, fmt='%.30f', delimiter=',')

